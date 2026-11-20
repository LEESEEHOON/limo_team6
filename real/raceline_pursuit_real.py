#!/usr/bin/env python3
"""[실물 LIMO용] 레이싱 라인 추종 + 라이다 기반 장애물 회피 노드 (LIMO Ackermann) — 계층형 구조.
(시뮬 코드 raceline_pursuit.py를 복사해서 출력/토픽/시계/속도만 실물에 맞게 바꾼 파일. 시뮬 코드는 그대로 둔다.)

구조 (계층형: 글로벌 → 로컬 → 추종, 세 단계가 매 주기 함께 동작)
  - Global path : Raceline-Optimization으로 미리 계산한 CSV. 출발 전에 한 번 읽어 주행 내내 유지.
  - Local path  : 장애물 유무와 상관없이 매 주기(20 Hz) 생성.
                  global path 앞 구간을 기준선으로, 좌우 오프셋 d 후보 경로들을 만들고
                  라이다 점과 차체 사각형으로 충돌 검사 → 비용 최소 경로 선택.
                  장애물이 없으면 d = 0 → local path = global path.
  - 추종        : 차는 항상 local path만 Pure Pursuit로 추종.
  - 보조 후보    : 끝까지 안전한 오프셋 경로가 없을 때(코너 안 장애물, 좁은 틈)는 같은 로컬 플래너 안에서
                  S자 조향 궤적 후보(목표는 막힌 구간 뒤 raceline 위의 점)를 쓴다.
  - 예외        : 모든 후보가 막혔을 때만 후진 복구.
  - [A] 지도 기반 장애물 구분: Q1 지도(maps/q1_grid.npz)에 있던 벽 근처 점은 '벽', 없던 점은 '장애물'.
        충돌 검사와 경로 비용은 벽+장애물 모두(벽에 붙지 않게), 장애물 옆 감속은 장애물만 본다.
  - [D] 속도: 제한값을 모아 가장 작은 값 하나만 쓴다.
        v = min(레이싱 라인 속도 프로파일, local path 곡률 제한, 장애물 옆 간격 제한(그 지점까지 감속 가능한 속도),
                지금 조향의 횡가속도 제한) → 가속만 a_long_max로 제한 (감속은 바로).

출력
  /limo/ack_cmd (ackermann_msgs/AckermannDrive: speed[m/s], steering_angle[rad]) → 리모 구동 노드가 바로 받음
                (output:=twist 로 바꾸면 /cmd_vel Twist를 내보냄. 시뮬에서 이 파일을 테스트할 때 사용)
  /plan       (Path) RViz 표시용 global path
  /local_plan (Path) RViz 표시용 local path (항상 발행)

설계 메모
  - 단일 스레드 executor + 논블로킹 TF 조회(timeout 0)로 교착 상태를 원천 차단.
  - 가장 가까운 경로점은 직전 위치 주변(window)만 검색 → 트랙 반대편 점으로 튀지 않는다.
  - 한 바퀴(경로 전체 길이) 진행하면 정지하고 랩타임을 출력한다.

사용법 (리모, bringup + 라이다 + SLAM/로컬라이제이션을 켠 뒤)
  python3 raceline_pursuit_real.py                          (build_raceline_real.py가 만든 q1_raceline_real.csv)
  python3 raceline_pursuit_real.py --ros-args -p v_max:=0.3 (속도 낮출 때)
  python3 raceline_pursuit_real.py --ros-args -p scan_topic:=/ydlidar/scan   (라이다 토픽 이름이 다를 때)
시뮬에서 이 파일을 테스트할 때
  python3 raceline_pursuit_real.py --ros-args -p use_sim_time:=true -p output:=twist -p wheelbase:=0.24 -p v_max:=1.0 -p v_avoid:=0.6 -p v_min:=0.25 -p a_lat_max:=0.8 -p a_long_max:=0.6 -p v_gap_min:=0.4 -p a_brake:=0.6
"""
import math
import csv
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_CSV = os.path.join(HERE, 'q1_raceline_real.csv')   # build_raceline_real.py가 이 스크립트와 같은 폴더에 저장


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RacelinePursuit(Node):
    def __init__(self):
        # 실물은 시스템 시계(use_sim_time=false). 시뮬에서 테스트할 때만 -p use_sim_time:=true
        super().__init__('raceline_pursuit')
        p = self.declare_parameter
        # 경로 / 차량
        self.csv_path = p('csv_path', '').value or REAL_CSV
        self.output = p('output', 'ackermann').value        # ackermann: /limo/ack_cmd | twist: /cmd_vel (시뮬 테스트용)
        self.ack_topic = p('ack_topic', '/limo/ack_cmd').value
        self.scan_topic = p('scan_topic', '/scan').value    # 라이다 launch를 켠 뒤 ros2 topic list로 확인해서 맞출 것
        self.scan_timeout = p('scan_timeout', 0.5).value    # 이 시간 동안 라이다가 안 오면 정지 [s]
        self.wheelbase = p('wheelbase', 0.20).value         # LIMO 축거 200 mm (시뮬은 0.24)
        self.max_steer = p('max_steer', 0.40).value        # 실제 한계 0.42보다 살짝 안쪽
        # 속도
        self.v_max = p('v_max', 0.4).value                 # 직선 최고 속도 (실물 첫 테스트는 낮게. 시뮬은 1.0)
        self.v_min = p('v_min', 0.12).value
        self.a_lat_max = p('a_lat_max', 0.5).value         # 코너 감속 기준 횡가속도 [m/s^2]
        self.a_long_max = p('a_long_max', 0.4).value       # 가감속 한계 (속도 프로파일 평활화)
        # Pure Pursuit
        self.ld_min = p('lookahead_min', 0.6).value
        self.ld_gain = p('lookahead_gain', 0.6).value      # Ld = ld_min + gain * v
        # 로컬 플래너 (항상 동작)
        self.local_len = p('local_length', 3.0).value           # local path 길이 [m]
        self.d_max = p('offset_max', 0.6).value                 # 좌우 오프셋 후보 범위 [m]
        self.d_step = p('offset_step', 0.1).value               # 오프셋 간격 [m]
        self.trans_min = p('transition_min', 1.2).value         # 오프셋 바꾸는 최소 거리 [m]
        self.w_offset = p('w_offset', 1.0).value                # 비용: raceline에서 벗어난 정도
        self.w_change = p('w_change', 0.5).value                # 비용: 직전 선택과의 차이 (흔들림 억제)
        self.w_clear = p('w_clear', 0.6).value                  # 비용: 장애물과 가까움
        self.clear_pref = p('clear_pref', 0.15).value           # 이 여유보다 가까우면 비용이 붙는다 [m]
        # 차체 사각형 (base_link = 앞뒤 바퀴 정중앙). LIMO 길이 ~0.32 m, 폭 ~0.213 m
        self.half_len = p('half_length', 0.16).value
        self.half_wid = p('half_width', 0.11).value
        self.margin = p('safety_margin', 0.03).value            # 사각형 바깥 여유
        self.v_avoid = p('v_avoid', 0.25).value                  # 보조 조향 궤적으로 빠져나갈 때 최고 속도
        # [D] 속도
        self.v_gap_min = p('v_gap_min', 0.15).value             # 장애물 옆 간격이 safety_margin뿐일 때 속도 [m/s]
        self.gap_full = p('gap_full', 0.30).value               # 장애물 옆 간격이 이 이상이면 장애물 때문에 감속 안 함 [m]
        self.a_brake = p('a_brake', 0.4).value                  # 미리 감속할 때 쓰는 감속도 [m/s^2]
        # [A] 벽 지도
        self.wall_map_path = p('wall_map', '').value or os.path.join(HERE, 'maps', 'q1_grid.npz')
        self.use_wall_map = p('use_wall_map', True).value
        self.wall_tol = p('wall_tol', 0.12).value               # 지도 벽에서 이 거리 안의 라이다 점은 벽으로 본다 [m]
        self.dwa_dist = p('dwa_dist', 1.6).value                # 보조 후보(조향 궤적) 예측 거리 [m]
        self.rejoin_clear = p('rejoin_clear', 0.5).value        # 복귀점: 장애물에서 이만큼 떨어진 경로점
        self.rejoin_extra = p('rejoin_extra', 0.8).value        # 그 점에서 경로를 따라 더 앞으로
        self.recovery_time = p('recovery_time', 1.5).value      # 막혔을 때 후진 시간 [s]
        self.reverse_speed = p('reverse_speed', 0.12).value     # 후진 속도 [m/s]
        # 기타
        self.laps = p('laps', 1).value                     # 0이면 무한 반복
        self.finish_extra = p('finish_extra', 0.45).value  # 출발선을 차 중심이 지난 뒤 더 갈 거리 (뒷바퀴까지 통과)
        self.rate = p('control_rate', 20.0).value

        self._load_path()
        self._load_wall_map()

        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        if self.output == 'ackermann':
            self.cmd_pub = self.create_publisher(AckermannDrive, self.ack_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, '/plan', qos_latched)
        self.local_pub = self.create_publisher(Path, '/local_plan', 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_pts = None          # base_link 기준 라이다 점 (N,2)
        self.last_scan = None         # 마지막 라이다 수신 시각 (끊기면 정지)
        self.idx = None               # 현재 경로 진행 인덱스
        self.progress = 0.0           # 누적 진행 거리 [m]
        self.start_time = None
        self.finished = False
        self.d_prev = 0.0             # 직전에 고른 목표 오프셋
        self.avoiding = False         # 로그용: 오프셋 ≠ 0 상태인가
        self.prev_steer = 0.0
        self.recovery_until = None    # 후진 복구 종료 시각
        self.recovery_steer = 0.0     # 후진할 때 조향 (앞머리를 빠져나갈 쪽으로 돌림)
        self.stuck_ref = None         # (시각, x, y): 전진 명령인데 안 움직이는지 감시
        self.cross_time = None        # 차 중심이 출발선을 다시 지난 시각
        self.raceline_blocked = False # 레이싱 라인(d=0) 후보가 막혔나
        self.v_cmd = 0.0              # 마지막으로 보낸 속도 (가속 제한용)
        self.min_gap = 9.0            # 주행 중 차체와 장애물 점 최소 간격 (완주 로그용)
        self.loop_ms = []             # 제어 한 번에 걸린 시간 [ms] (완주 로그용)

        self.create_timer(1.0 / self.rate, self._control)
        self.create_timer(1.0, self._publish_plan)
        self.get_logger().info(
            f'경로 {self.n}점, 길이 {self.total_len:.1f} m, v_max {self.v_max} m/s, '
            f'로컬 플래너 상시 동작 (오프셋 ±{self.d_max} m), 출력 {self.output}'
            + (f' → {self.ack_topic}' if self.output == 'ackermann' else ' → /cmd_vel') + f', 라이다 {self.scan_topic}')

    # ------------------------------------------------------------------ 경로
    def _load_path(self):
        if not os.path.exists(self.csv_path):
            raise SystemExit(f'경로 파일이 없습니다: {self.csv_path}\n'
                             '먼저 build_raceline_real.py로 매핑하며 경로를 만드세요 (또는 -p csv_path:=파일).')
        pts = []
        with open(self.csv_path) as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                pts.append((float(row[0]), float(row[1])))
        xy = np.array(pts)
        if np.hypot(*(xy[0] - xy[-1])) < 1e-3:   # 마지막 점이 시작점과 같으면 제거 (루프로 다룸)
            xy = xy[:-1]
        self.xy = xy
        self.n = len(xy)
        seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
        self.seg = seg                                    # i → i+1 거리
        self.total_len = float(seg.sum())
        self.s_cum = np.concatenate([[0.0], np.cumsum(seg)])   # 루프 보간용 누적 거리 (n+1개)

        # 곡률 (세 점 외접원)
        k = np.zeros(self.n)
        for i in range(self.n):
            a, b, c = xy[i - 1], xy[i], xy[(i + 1) % self.n]
            ab, bc, ac = np.hypot(*(b - a)), np.hypot(*(c - b)), np.hypot(*(c - a))
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            k[i] = 2 * abs(cross) / (ab * bc * ac) if ab * bc * ac > 1e-9 else 0.0
        self.kappa = k

        # 속도 프로파일: 횡가속도 한계 → 앞/뒤로 가감속 한계 평활화 (루프라 두 바퀴 돌림)
        v = np.minimum(self.v_max, np.sqrt(self.a_lat_max / np.maximum(k, 1e-6)))
        v = np.maximum(v, self.v_min)
        for _ in range(2):
            for i in range(self.n):        # 가속 한계 (앞으로)
                j = (i + 1) % self.n
                v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * self.a_long_max * seg[i]))
            for i in range(self.n - 1, -1, -1):  # 감속 한계 (뒤로)
                j = (i + 1) % self.n
                v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * self.a_long_max * seg[i]))
        self.v_ref = v

    # ------------------------------------------------------------------ [A] 벽 지도
    def _load_wall_map(self):
        """Q1 지도에서 '벽 근처' 칸을 미리 표시해 둔다. 지도에 없던 라이다 점만 장애물로 보기 위함."""
        self.wall = None
        if not self.use_wall_map:
            self.get_logger().info('벽 지도 끔 → 모든 라이다 점을 장애물로 취급')
            return
        if not os.path.exists(self.wall_map_path):
            self.get_logger().warn(f'벽 지도 없음({self.wall_map_path}) → 모든 라이다 점을 장애물로 취급 (기존 방식)')
            return
        z = np.load(self.wall_map_path)
        occ, free = z['occ'].astype(bool), z['free'].astype(bool)
        self.wall_res, self.wall_ox, self.wall_oy = float(z['res']), float(z['ox']), float(z['oy'])
        h, w = occ.shape
        r = int(math.ceil(self.wall_tol / self.wall_res))
        pad = np.pad(occ, r)
        near = np.zeros_like(occ)
        for dy in range(-r, r + 1):              # 벽 칸을 wall_tol만큼 두껍게 (위치추정 오차 흡수)
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    near |= pad[r + dy:r + dy + h, r + dx:r + dx + w]
        self.wall = near | ~(occ | free)         # 벽 근처 + 매핑 때 못 본 곳(트랙 밖)
        self.get_logger().info(f'벽 지도 사용: {self.wall_map_path} ({w}x{h}, 벽 여유 {self.wall_tol:.2f} m)')

    def _is_wall(self, pm):
        """map 좌표 점들이 지도의 벽(근처)인지. 지도 밖은 벽으로 본다. 벽 지도가 없으면 전부 False(=장애물)."""
        out = np.zeros(len(pm), dtype=bool)
        if self.wall is None or not len(pm):
            return out
        c = np.floor((pm[:, 0] - self.wall_ox) / self.wall_res).astype(int)
        r = np.floor((pm[:, 1] - self.wall_oy) / self.wall_res).astype(int)
        h, w = self.wall.shape
        ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        out[:] = True
        out[ok] = self.wall[r[ok], c[ok]]
        return out

    def _obstacles(self, x, y, yaw):
        """라이다 점 → (base_link 점, map 점, 장애물인가). 가까운 점만, 최대 300개."""
        ob = self.scan_pts if self.scan_pts is not None else np.empty((0, 2))
        if len(ob):
            ob = ob[np.hypot(ob[:, 0], ob[:, 1]) < max(self.local_len, self.dwa_dist) + 0.8]
            if len(ob) > 300:
                ob = ob[::int(math.ceil(len(ob) / 300))]
        c, s = math.cos(yaw), math.sin(yaw)
        pm = np.column_stack([x + c * ob[:, 0] - s * ob[:, 1], y + s * ob[:, 0] + c * ob[:, 1]])
        return ob, pm, ~self._is_wall(pm)

    def _publish_plan(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in np.vstack([self.xy, self.xy[:1]]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.plan_pub.publish(msg)

    # ------------------------------------------------------------------ 센서
    def _lookup(self, target, source):
        """논블로킹 TF 조회. 없으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_quat(t.transform.rotation)

    def _on_scan(self, msg: LaserScan):
        tf = self._lookup('base_link', msg.header.frame_id)
        if tf is None:
            self.get_logger().warn(f'TF {msg.header.frame_id}→base_link 없음 (라이다 프레임 TF 확인)',
                                   throttle_duration_sec=3.0)
            return
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, 4.0))
        r, ang = r[ok], ang[ok]
        lx, ly = r * np.cos(ang), r * np.sin(ang)
        ox, oy, oyaw = tf
        c, s = math.cos(oyaw), math.sin(oyaw)
        self.scan_pts = np.column_stack([ox + c * lx - s * ly, oy + s * lx + c * ly])
        self.last_scan = self.get_clock().now()

    # ------------------------------------------------------------------ 경로 진행
    def _update_index(self, x, y):
        if self.idx is None:      # 처음엔 전체에서 검색
            self.idx = int(np.argmin(np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)))
            return 0.0
        ids = (self.idx + np.arange(-5, 40)) % self.n   # 직전 위치 주변만 검색
        d = np.hypot(self.xy[ids, 0] - x, self.xy[ids, 1] - y)
        new = int(ids[int(np.argmin(d))])
        step = (new - self.idx) % self.n
        if step > self.n // 2:    # 뒤로 간 경우 (거의 없음)
            step = 0
            new = self.idx
        advanced = float(sum(self.seg[(self.idx + i) % self.n] for i in range(step)))
        self.idx = new
        return advanced

    def _point_ahead(self, dist):
        """현재 인덱스에서 경로를 따라 dist[m] 앞의 점과 인덱스."""
        i, acc = self.idx, 0.0
        while acc < dist:
            acc += self.seg[i]
            i = (i + 1) % self.n
        return self.xy[i], i

    def _reference(self, x, y, length, ds=0.1):
        """global path에서 로봇 위치에 해당하는 지점부터 length[m]를 ds 간격으로 다시 뽑은 기준선.
        반환: 점(M,2), 단위 접선(M,2), 왼쪽 법선(M,2), 로봇의 현재 횡오프셋 d0 (왼쪽 +)."""
        # 현재 인덱스 구간 위로 로봇을 투영해서 시작 거리 s0를 정한다
        i, j = self.idx, (self.idx + 1) % self.n
        a, b = self.xy[i], self.xy[j]
        t_vec = b - a
        seg = max(float(np.hypot(*t_vec)), 1e-6)
        u = float(np.dot([x - a[0], y - a[1]], t_vec)) / seg
        s0 = self.s_cum[i] + min(max(u, -seg), 2 * seg)
        s = (s0 + np.arange(0.0, length + 1e-6, ds)) % self.total_len
        loop = np.vstack([self.xy, self.xy[:1]])
        px = np.interp(s, self.s_cum, loop[:, 0])
        py = np.interp(s, self.s_cum, loop[:, 1])
        ref = np.column_stack([px, py])
        tang = np.gradient(ref, axis=0)
        tang /= np.maximum(np.hypot(tang[:, 0], tang[:, 1]), 1e-9)[:, None]
        nrm = np.column_stack([-tang[:, 1], tang[:, 0]])
        d0 = float(np.dot([x - ref[0, 0], y - ref[0, 1]], nrm[0]))
        return ref, tang, nrm, d0

    # ------------------------------------------------------------------ 제어
    def _control(self):
        if self.finished:
            return
        pose = self._lookup('map', 'base_link')
        if pose is None:
            self.get_logger().info('map→base_link TF 대기 중...', throttle_duration_sec=2.0)
            return
        x, y, yaw = pose
        # 안전: 라이다가 없거나 끊겼으면 움직이지 않는다 (실물)
        if self.last_scan is None or (self.get_clock().now() - self.last_scan) > Duration(seconds=self.scan_timeout):
            self.get_logger().warn(f'{self.scan_topic} 라이다 수신 없음 → 정지 대기', throttle_duration_sec=2.0)
            self._stop()
            return
        self.progress += self._update_index(x, y)
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(f'출발! 시작 인덱스 {self.idx}')

        if self.laps > 0 and self.progress >= self.laps * self.total_len:
            if self.cross_time is None:      # 차 중심(base_link)이 출발선을 지난 순간
                self.cross_time = self.get_clock().now()
            if self.progress >= self.laps * self.total_len + self.finish_extra:
                self._finish()               # 차 전체(뒷바퀴까지)가 출발선을 넘어감
                return

        v_target = float(self.v_ref[self.idx])

        # 후진 복구 중이면 그대로 후진
        if self.recovery_until is not None:
            if self.get_clock().now() < self.recovery_until:
                if self._reverse_clear(self.recovery_steer):
                    self._send(-self.reverse_speed, self.recovery_steer)
                else:                                   # 뒤도 막힘 → 후진 중단
                    self.recovery_until = None
                    self._send(0.0, 0.0)
                return
            self.recovery_until = None
            self.get_logger().info('후진 복구 끝 → 로컬 경로 다시 생성')

        t0 = time.perf_counter()
        dec = self._decide(x, y, yaw)
        self.loop_ms.append((time.perf_counter() - t0) * 1000.0)
        # 끼임 감시: 전진 명령을 내는데 2.5초 동안 5 cm도 못 움직였으면 막힌 것으로 본다
        now = self.get_clock().now()
        if self.stuck_ref is None or math.hypot(x - self.stuck_ref[1], y - self.stuck_ref[2]) > 0.05:
            self.stuck_ref = (now, x, y)
        elif dec is not None and (now - self.stuck_ref[0]) > Duration(seconds=2.5):
            self.get_logger().warn('전진 명령인데 움직이지 않음 (장애물에 걸림)')
            self.stuck_ref = (now, x, y)
            dec = None
        if dec is None:   # 모든 후보가 막힘 → 방향 틀며 후진
            self.recovery_steer = self._choose_reverse_steer(x, y, yaw)
            if self.recovery_steer is None:
                self.get_logger().warn('앞뒤 모두 막힘 → 정지 대기', throttle_duration_sec=1.0)
                self._send(0.0, 0.0)
                return
            self.get_logger().warn(f'로컬 경로 없음 → 후진 복구 (조향 {self.recovery_steer:+.2f})')
            self.recovery_until = self.get_clock().now() + Duration(seconds=self.recovery_time)
            self._send(-self.reverse_speed, self.recovery_steer)
            return
        v, steer, kind, d_sel = dec

        # 로그: local path가 raceline에서 벗어나고 돌아올 때만 알림 (모드 전환이 아니라 local path 모양 변화)
        off = kind == 'steer' or abs(d_sel) > 0.05
        if off and not self.avoiding:
            self.avoiding = True
            how = '조향 궤적 후보' if kind == 'steer' else f'오프셋 d={d_sel:+.1f} m'
            self.get_logger().warn(f'장애물 → local path가 raceline에서 비켜감 ({how})')
        elif not off and self.avoiding:
            self.avoiding = False
            self.get_logger().info('장애물 통과 → local path = raceline (d=0)')
        self.prev_steer = steer
        self._send(v, steer)

    def _decide(self, x, y, yaw):
        """로컬 플래너 → 추종. 반환 (v, steer, 종류, d) 또는 None(모든 후보 막힘).
        종류 'offset': raceline 오프셋 경로를 Pure Pursuit로 추종 / 'steer': 보조 조향 궤적을 그대로 따름."""
        v_target = float(self.v_ref[self.idx])
        ob, pm, is_obs = self._obstacles(x, y, yaw)
        if len(ob):     # 차체와 가장 가까운 장애물 점 (기록용)
            here = self._box_clearance(np.zeros(1), np.zeros(1), np.zeros(1), ob[is_obs]) if is_obs.any() else 9.0
            self.min_gap = min(self.min_gap, here)
        res = self._local_plan(x, y, yaw, ob, pm, is_obs)
        if res is None:
            return None
        kind, a, d_sel, info = res
        if kind == 'steer':
            # 보조 조향 궤적: 좁은 곳을 빠져나가는 중이라 천천히
            clear = info
            scale = min(1.0, max(0.0, (clear - self.margin) / (self.clear_pref - self.margin)))
            v_target = max(self.v_min, min(v_target, self.v_avoid) * (0.6 + 0.4 * scale))
            steer = a
            v = max(self.v_min, v_target * (1.0 - 0.3 * abs(steer) / self.max_steer))
        else:
            # [D] local path를 따라 미리 본 허용 속도 (곡률, 장애물 옆 간격)
            v_target = min(v_target, self._preview_speed(a, info))
            steer, v = self._pure_pursuit(a, x, y, yaw, v_target)
        # [D] 가속만 제한 (감속은 바로 반영)
        v = min(v, max(self.v_cmd, 0.0) + self.a_long_max / self.rate)
        return v, steer, kind, d_sel

    def _local_plan(self, x, y, yaw, ob, pm, is_obs):
        """로컬 경로 생성 (매 주기). 반환 ('offset', 경로(M,2), d, 자세별 장애물 간격(M,)) / ('steer', 조향, 0, 여유) / None.
        1순위 후보: raceline 오프셋 경로. d(s)는 현재 횡오프셋 d0에서 목표 d_t까지 부드럽게(smoothstep) 옮긴 뒤 유지.
        충돌 검사와 경로 비용(여유)은 벽+장애물 모든 점 (벽에 붙지 않게, 기존과 같음).
        [A] 속도 제한(장애물 옆 간격)에만 장애물 점만 쓴다 → 벽 옆에서 쓸데없이 감속하지 않음.
        끝까지 안전한 오프셋 후보가 없을 때만 2순위 후보(S자 조향 궤적, 복귀 목표는 raceline 위)를 쓴다."""
        targets = np.round(np.arange(-self.d_max, self.d_max + 1e-6, self.d_step), 3)
        ref, tang, nrm, d0 = self._reference(x, y, self.local_len)
        sl = np.arange(len(ref)) * 0.1
        heading = np.arctan2(tang[:, 1], tang[:, 0])
        n_obs = int(is_obs.sum())
        far = np.full(len(ref), 5.0)
        best = None                  # 끝까지 안 막힌 후보 중 비용 최소
        for d_t in targets:
            # 옮기는 양이 클수록 길게 (smoothstep 최대 곡률 6·Δd/T² ≤ 1.5 → LIMO가 돌 수 있게)
            trans = max(self.trans_min, math.sqrt(4.0 * abs(d_t - d0)))
            r = np.clip(sl / trans, 0.0, 1.0)
            d = d0 + (d_t - d0) * (3 * r ** 2 - 2 * r ** 3)       # smoothstep
            path = ref + nrm * d[:, None]
            gap, gap_all = far, far
            if len(pm):
                # 지금 차 위치부터 전부 검사 (옆에 붙은 장애물도 보이게)
                dist = self._box_dist(path[:, 0], path[:, 1], heading, pm)     # (자세, 점)
                hit = bool((dist.min(axis=1) < self.margin).any())             # 벽이든 장애물이든 닿으면 못 씀
                if d_t == 0.0:
                    self.raceline_blocked = hit
                if hit:
                    continue
                gap_all = dist.min(axis=1)                                     # 벽 포함 (경로 비용용)
                if n_obs:
                    gap = dist[:, is_obs].min(axis=1)                         # 자세별 장애물 간격 (속도 제한용, 벽 제외)
            clear = float(gap_all.min())
            cost = (self.w_offset * abs(d_t) + self.w_change * abs(d_t - self.d_prev)
                    + self.w_clear * max(0.0, self.clear_pref - clear) / self.clear_pref)
            if best is None or cost < best[0]:
                best = (cost, path, float(d_t), gap)
        if best is None:
            # 끝까지 안전한 오프셋이 없음(코너 안 장애물, 좁은 틈 등) → 보조 후보: 조향 궤적
            res = self._steer_candidates(x, y, yaw, ob, is_obs)
            if res is None:
                return None
            steer, traj, clear = res
            self.d_prev = 0.0
            self._publish_local(traj)
            return 'steer', steer, 0.0, clear
        _, path, d_sel, gap = best
        self.d_prev = d_sel
        self._publish_local(path)
        return 'offset', path, d_sel, gap

    def _preview_speed(self, path, gap):
        """[D] local path 위 각 지점(0.1 m 간격)의 허용 속도 = min(곡률 제한, 장애물 옆 간격 제한).
        지금 낼 수 있는 속도 = 그 지점까지 a_brake로 감속해서 맞출 수 있는 속도 중 최솟값."""
        n = len(path)
        k = np.zeros(n)
        j = np.arange(3, n - 3)
        if len(j):
            a, b, c = path[j - 3], path[j], path[j + 3]
            ab, bc, ac = np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(c - a).T)
            cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
            k[j] = 2 * np.abs(cross) / np.maximum(ab * bc * ac, 1e-9)
        v_curv = np.sqrt(self.a_lat_max / np.maximum(k, 1e-6))
        ratio = np.clip((gap - self.margin) / (self.gap_full - self.margin), 0.0, 1.0)
        v_gap = self.v_gap_min + (self.v_max - self.v_gap_min) * ratio
        v_allow = np.minimum(np.minimum(v_curv, v_gap), self.v_max)
        s = np.arange(n) * 0.1
        return float(np.min(np.sqrt(v_allow ** 2 + 2.0 * self.a_brake * s)))

    def _steer_candidates(self, x, y, yaw, ob, is_obs):
        """보조 후보: 조향 11×11 S자 궤적을 차체 사각형으로 충돌 검사하고, 막힌 raceline 구간 뒤의
        복귀 목표에 가장 잘 가는 궤적을 고른다. 반환 (첫 조향, 궤적(map 좌표), 여유) 또는 None."""
        sel = np.hypot(ob[:, 0], ob[:, 1]) < self.dwa_dist + 0.8 if len(ob) else np.zeros(0, dtype=bool)
        obs = ob[sel]
        goal = self._rejoin_target(x, y, yaw, ob[sel & is_obs] if len(ob) else ob)   # [A] 복귀점은 장애물만 보고
        steers = np.linspace(-self.max_steer, self.max_steer, 11)
        best = None
        # 1.6 m 궤적이 전부 막히면 더 짧은 궤적으로 다시 찾는다 (좁은 코너에서 조금씩 전진)
        for dist in (self.dwa_dist, 1.0, 0.6):
            d1 = min(0.5, dist / 2)
            for s1 in steers:
                for s2 in steers:
                    tx, ty, th = self._rollout(s1, s2, d1, 0.1, dist)
                    clear = self._box_clearance(tx, ty, th, obs) if len(obs) else 5.0
                    if clear < self.margin:
                        continue
                    cost = (math.hypot(goal[0] - tx[-1], goal[1] - ty[-1])
                            + 0.08 / (clear + 0.05) + 0.15 * abs(s1 - self.prev_steer))
                    if best is None or cost < best[0]:
                        best = (cost, float(s1), np.column_stack([tx, ty]), clear)
            if best is not None:
                break
        if best is None:
            return None
        _, s1, traj, clear = best
        c, s = math.cos(yaw), math.sin(yaw)
        traj_map = np.column_stack([x + c * traj[:, 0] - s * traj[:, 1], y + s * traj[:, 0] + c * traj[:, 1]])
        return s1, traj_map, clear

    def _rejoin_target(self, x, y, yaw, obs):
        """장애물에 막힌 raceline 구간을 지난 뒤, 처음으로 비어 있는 경로점 + 여유거리 (base_link 좌표)."""
        c, s = math.cos(-yaw), math.sin(-yaw)
        i, acc, seen_blocked, clear_at = self.idx, 0.0, False, None
        while acc < 6.0:
            dx, dy = self.xy[i, 0] - x, self.xy[i, 1] - y
            pb = (c * dx - s * dy, s * dx + c * dy)
            if pb[0] > 0.2 and len(obs):
                d = float(np.min(np.hypot(obs[:, 0] - pb[0], obs[:, 1] - pb[1])))
                if d < self.rejoin_clear:
                    seen_blocked, clear_at = True, None
                elif seen_blocked and clear_at is None:
                    clear_at = acc
            if clear_at is not None and acc >= clear_at + self.rejoin_extra:
                return pb
            acc += self.seg[i]
            i = (i + 1) % self.n
        (gx, gy), _ = self._point_ahead(2.5)          # 못 찾으면 2.5 m 앞
        return (c * (gx - x) - s * (gy - y), s * (gx - x) + c * (gy - y))

    def _pure_pursuit(self, local, x, y, yaw, v_target):
        ld = self.ld_min + self.ld_gain * v_target
        # local path 위에서 로봇과 가장 가까운 점부터 ld만큼 앞의 점
        i0 = int(np.argmin(np.hypot(local[:, 0] - x, local[:, 1] - y)))
        j = min(i0 + int(round(ld / 0.1)), len(local) - 1)
        gx, gy = local[j]
        dx, dy = gx - x, gy - y
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        alpha = math.atan2(ly, lx)
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), max(math.hypot(lx, ly), 1e-3))
        steer = max(-self.max_steer, min(self.max_steer, steer))
        # [D] 지금 조향으로 도는 곡률의 횡가속도 한계 (예전: 핸들 각도에 비례해 최대 50% 감속 → 코너 이중 감속)
        k_cmd = abs(math.tan(steer)) / self.wheelbase
        v = min(v_target, math.sqrt(self.a_lat_max / max(k_cmd, 1e-6)))
        return steer, max(self.v_min, v)

    def _rollout(self, s1, s2, d1, ds, dist):
        """앞 d1[m]는 조향 s1, 나머지는 s2로 가는 궤적 (base_link 기준 x, y, heading).
        dist가 음수면 후진 궤적."""
        sign = -1.0 if dist < 0 else 1.0
        n = max(1, int(abs(dist) / ds))
        ds = sign * ds
        k = np.where(np.arange(n) * abs(ds) < d1, math.tan(s1), math.tan(s2)) / self.wheelbase
        th = np.cumsum(k * ds)
        return np.cumsum(np.cos(th) * ds), np.cumsum(np.sin(th) * ds), th

    def _box_dist(self, tx, ty, th, obs):
        """궤적 자세(P)마다 차체 사각형과 점(N) 사이 거리 (P,N) [m]. 음수면 사각형 안(충돌)."""
        dx = obs[None, :, 0] - tx[:, None]
        dy = obs[None, :, 1] - ty[:, None]
        c, s = np.cos(th)[:, None], np.sin(th)[:, None]
        lx = np.abs(c * dx + s * dy) - self.half_len     # 차체 좌표로 옮긴 뒤 사각형 밖으로 얼마나 떨어졌나
        ly = np.abs(-s * dx + c * dy) - self.half_wid
        outside = np.hypot(np.maximum(lx, 0), np.maximum(ly, 0))
        inside = np.minimum(np.maximum(lx, ly), 0)
        return outside + inside

    def _box_clearance(self, tx, ty, th, obs, per_pose=False):
        """궤적 위 각 자세에서 차체 사각형과 장애물 점 사이 최소 여유[m]. 음수면 사각형 안(충돌).
        per_pose=True면 자세별 여유 배열을 돌려준다."""
        d = self._box_dist(tx, ty, th, obs)
        return d.min(axis=1) if per_pose else float(d.min())

    def _reverse_clear(self, steer, dist=0.35):
        """조향 steer로 dist[m] 후진했을 때 뒤쪽이 비어 있나 (base_link 기준으로 검사)."""
        obs = self.scan_pts if self.scan_pts is not None else np.empty((0, 2))
        if not len(obs):
            return True
        tx, ty, th = self._rollout(steer, steer, 1.0, 0.05, -dist)
        return self._box_clearance(tx, ty, th, obs) >= self.margin

    def _choose_reverse_steer(self, x, y, yaw):
        """후진하면서 앞머리가 global path 1.5 m 앞 점 쪽으로 돌도록 조향을 고른다.
        (후진할 땐 핸들을 반대로 꺾어야 앞머리가 그쪽으로 돈다) 뒤가 다 막혔으면 None."""
        (gx, gy), _ = self._point_ahead(1.5)
        gyb = math.sin(-yaw) * (gx - x) + math.cos(-yaw) * (gy - y)   # 목표의 로봇 기준 좌우 위치
        want = -math.copysign(1.0, gyb) if abs(gyb) > 0.05 else 0.0
        for steer in (want * self.max_steer, want * 0.5 * self.max_steer, 0.0,
                      -want * 0.5 * self.max_steer):
            if self._reverse_clear(steer):
                return steer
        return None

    # ------------------------------------------------------------------ 유틸
    def _publish_local(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.local_pub.publish(msg)

    def _send(self, v, steer):
        self.v_cmd = float(v)
        if self.output == 'ackermann':
            cmd = AckermannDrive()
            cmd.speed = float(v)
            cmd.steering_angle = float(steer)     # 후진(v<0)일 때도 steer 그대로 (시뮬 어댑터와 같은 규칙)
            self.cmd_pub.publish(cmd)
            return
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(v * math.tan(steer) / self.wheelbase)  # 어댑터가 다시 조향각으로 변환
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self._send(0.0, 0.0)

    def _finish(self):
        self.finished = True
        for _ in range(5):
            self._stop()
        now = self.get_clock().now()
        lap_c = (self.cross_time - self.start_time).nanoseconds / 1e9
        lap_f = (now - self.start_time).nanoseconds / 1e9
        self.get_logger().info(f'완주! 진행 거리 {self.progress:.1f} m, 랩타임 {lap_c:.1f} s '
                               f'(차 중심 출발선 통과) / {lap_f:.1f} s (차 전체 통과)')
        if self.loop_ms:
            ms = np.array(self.loop_ms)
            self.get_logger().info(f'장애물 최소 간격(라이다 기준) {self.min_gap * 100:.1f} cm, 제어 계산 평균 {ms.mean():.1f} ms '
                                   f'/ 최대 {ms.max():.1f} ms (주기 {1000.0 / self.rate:.0f} ms)')


def main():
    rclpy.init()
    node = RacelinePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
