#!/usr/bin/env python3
"""[실물 LIMO용] Q1(새 트랙)용: 매핑하면서 지나간 길을 기록해서, 한 바퀴가 끝나면 레이싱 라인을 자동으로 만든다.

흐름
  1) SLAM(매핑) 중에 map→base_link 위치를 계속 기록
  2) 10 m 이상 달린 뒤 아래 중 하나면 "한 바퀴 완료"
       - 출발점 1.0 m 안으로 들어옴
       - 출발점 2.0 m 안을 지나가다가 가장 가까웠던 거리보다 0.3 m 다시 멀어짐 (출발점 옆을 스쳐 지나감)
       - 출발점 2.0 m 안에서 3초 이상 멈춤
  3) 최신 /map(OccupancyGrid)을 받아서
       - 기록 경로를 매끄럽게 하고, 좌우 벽 거리 차이만큼 옮겨서 트랙 중앙선으로 보정
       - 중앙선의 좌우 폭 측정 → Raceline-Optimization(최소 곡률) 입력 CSV 작성
       - 최적화 실행 → 결과 검사(벽과 거리, 곡률)
  4) 결과를 q1_raceline_real.csv로 이 스크립트와 같은 폴더에 저장 (최적화 실패/불량이거나 최적화 도구가 없으면
     보정한 중앙선을 대신 저장)
  5) 로컬라이제이션(Q2)용으로 slam_toolbox 지도(포즈그래프)를 maps/q1_map.* 에 저장
     + 장애물 구분용 벽 지도를 maps/q1_grid.npz 에 저장 (raceline_pursuit_real.py가 읽음)
     → raceline_pursuit_real.py 로 주행

사용법 (SLAM 매핑 켠 상태에서)
  python3 build_raceline_real.py
  한 바퀴 다 돌기 전에 끝내고 싶으면 Ctrl+C (10 m 이상 기록됐으면 그걸로 만든다)
"""
import glob
import math
import os
import re
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
# Raceline-Optimization과 그 파이썬 환경은 노트북(도커)에만 있다. 없으면 최적화를 건너뛰고 보정한 중앙선을 저장한다.
OPT_DIR = os.environ.get('RACELINE_OPT_DIR', '/home/seehoon/Documents/ws_limo_humble/src/limo_ros2/'
                                             'limo_slam/raceline_tool/Raceline-Optimization')
VENV_PY = os.environ.get('RACELINE_VENV_PY', '/opt/raceline/venv/bin/python')
OUT_CSV = os.path.join(HERE, 'q1_raceline_real.csv')
OUT_PNG = os.path.join(HERE, 'q1_raceline_real.png')
MAP_BASE = os.path.join(HERE, 'maps', 'q1_map')     # 확장자 없이. mapper_params_localization.yaml의 map_file_name과 같아야 함
TRACK_NAME = 'q1_track'


# ---------------------------------------------------------------------- 기하 유틸
def resample(xy, step):
    """폐곡선을 step[m] 간격으로 다시 찍는다."""
    closed = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    t = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])])


def smooth(xy, win):
    """폐곡선 이동평균 (원형)."""
    if win < 2:
        return xy
    k = np.ones(win) / win
    pad = np.vstack([xy[-win:], xy, xy[:win]])
    return np.column_stack([np.convolve(pad[:, i], k, mode='same')[win:-win] for i in range(2)])


def smooth1d(v, win):
    """원형(폐곡선) 1차원 이동평균."""
    if win < 2:
        return v
    k = np.ones(win) / win
    pad = np.concatenate([v[-win:], v, v[:win]])
    return np.convolve(pad, k, mode='same')[win:-win]


def normals(xy):
    d = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    d /= np.maximum(np.hypot(d[:, 0], d[:, 1])[:, None], 1e-9)
    return np.column_stack([-d[:, 1], d[:, 0]])            # 진행 방향 왼쪽


def curvature(xy):
    a, b, c = np.roll(xy, 1, axis=0), xy, np.roll(xy, -1, axis=0)
    ab, bc, ac = (np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(c - a).T))
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    return 2 * np.abs(cross) / np.maximum(ab * bc * ac, 1e-9)


class Grid:
    """OccupancyGrid 래퍼. free = 확실히 비어 있는 칸만 (unknown은 벽 취급)."""

    def __init__(self, msg: OccupancyGrid):
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.w, self.h = msg.info.width, msg.info.height
        data = np.asarray(msg.data, dtype=np.int16).reshape(self.h, self.w)
        self.free = (data >= 0) & (data < 50)
        self.occ = data >= 50

    def is_free(self, x, y):
        c, r = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        return 0 <= r < self.h and 0 <= c < self.w and bool(self.free[r, c])

    def ray(self, x, y, nx, ny, max_d=3.0):
        t = 0.0
        while t < max_d and self.is_free(x + nx * t, y + ny * t):
            t += self.res / 2
        return t

    def widths(self, xy, max_d=3.0):
        n = normals(xy)
        L = np.array([self.ray(p[0], p[1], q[0], q[1], max_d) for p, q in zip(xy, n)])
        R = np.array([self.ray(p[0], p[1], -q[0], -q[1], max_d) for p, q in zip(xy, n)])
        return L, R, n


# ---------------------------------------------------------------------- 레이싱 라인 생성
def build(path, grid: Grid, log):
    # 1. 기록 경로 정리: 0.2 m 간격 + 이동평균
    xy = smooth(resample(path, 0.2), 7)

    # 2. 중앙선으로 보정: 좌우 벽 거리 차이의 절반만큼 옆으로 옮긴다.
    #    넓게 움푹 들어간 공간에 끌려가지 않게 벽 거리는 2 m까지만 보고,
    #    옮기는 양은 ±0.2 m로 제한 + 앞뒤 2 m 평균 → 4번에 걸쳐 천천히 중앙으로.
    for _ in range(4):
        L, R, n = grid.widths(xy, max_d=2.0)
        shift = smooth1d(np.clip((L - R) / 2, -0.2, 0.2), 11)
        xy = smooth(resample(xy + n * shift[:, None], 0.2), 5)
    # 너무 급하게 꺾인 곳(회전반경 0.4 m 미만)이 남아 있으면 더 다듬는다
    for _ in range(10):
        if curvature(xy).max() < 2.5:
            break
        xy = smooth(xy, 5)
    L0, R0, _ = grid.widths(xy)
    log(f'중앙선 {len(xy)}점, 길이 {len(xy) * 0.2:.1f} m, '
        f'트랙 폭 최소 {(L0 + R0).min():.2f} m / 평균 {(L0 + R0).mean():.2f} m')
    centerline = xy.copy()
    a, b, c = np.roll(xy, 1, axis=0), xy, np.roll(xy, -1, axis=0)
    turn = np.sign((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    radius = 1.0 / np.maximum(smooth1d(curvature(xy), 5), 1e-3)

    # 3~5. 최적화. 최적화 도구는 좌우로 폭만큼 뻗은 법선이 서로 교차하면 거부하므로,
    #      실패하면 알려주는 폭 상한을 줄여서 다시 시도한다 (폭을 줄이는 쪽이라 안전).
    final, kind, why = centerline, '중앙선(최적화 실패 시 대체)', ''
    have_opt = os.path.isdir(OPT_DIR) and os.path.exists(VENV_PY)
    if not have_opt:
        log('Raceline-Optimization 환경이 없음 → 최적화를 건너뛰고 보정한 중앙선을 저장합니다 '
            '(노트북 도커에서 만들면 최적 라인)')
    for cap_max in ((1.2, 0.9, 0.7, 0.5) if have_opt else ()):
        cap_in = np.minimum(cap_max, 0.8 * radius)                       # 코너 안쪽은 반경의 80%까지
        L = np.where(turn > 0, np.minimum(L0, cap_in), np.minimum(L0, cap_max))   # 왼쪽으로 돌면 왼쪽이 안쪽
        R = np.where(turn < 0, np.minimum(R0, cap_in), np.minimum(R0, cap_max))
        width = L + R
        width_opt = float(np.clip(np.percentile(width, 5) - 0.4, 0.35, 0.8))  # 차폭+여유, 최소 0.2 m는 움직일 여지
        ini = f'{OPT_DIR}/params/f110.ini'
        text = re.sub(r'optim_opts_mincurv=\{"width_opt": [0-9.]+,',
                      f'optim_opts_mincurv={{"width_opt": {width_opt:.2f},', open(ini).read())
        open(ini, 'w').write(text)
        with open(f'{OPT_DIR}/inputs/tracks/{TRACK_NAME}.csv', 'w') as f:
            f.write('# x_m,y_m,w_tr_right_m,w_tr_left_m\n')
            for (x, y), r, l in zip(xy, R, L):
                f.write(f'{x:.4f},{y:.4f},{r:.4f},{l:.4f}\n')
        log(f'Raceline-Optimization 실행 (폭 상한 {cap_max} m, width_opt {width_opt:.2f}) ...')

        raceline = None
        try:
            env = dict(os.environ, MPLBACKEND='Agg')
            proc = subprocess.run([VENV_PY, 'main_globaltraj_f110.py', '--map_name', TRACK_NAME],
                                  cwd=OPT_DIR, env=env, capture_output=True, text=True, timeout=180)
            outs = sorted(glob.glob(f'{OPT_DIR}/outputs/{TRACK_NAME}/traj_race_cl-*.csv'),
                          key=os.path.getmtime)
            if outs and 'Finished export' in proc.stdout:
                raceline = np.loadtxt(outs[-1], delimiter=',')[:, :2]
            else:
                why = ((proc.stdout + proc.stderr).strip().splitlines() or ['출력 없음'])[-1]
        except Exception as e:     # noqa: BLE001
            why = str(e)
        if raceline is None:
            log(f'  실패: {why}')
            continue

        # 결과 검사: 벽과 0.15 m 이상 떨어져 있고, 곡률이 LIMO 한계(반경 0.59 m) 안쪽인지
        rl = resample(raceline, 0.2)
        rL, rR, _ = grid.widths(rl)
        kmax = float(curvature(rl).max())
        ok_wall = float(np.minimum(rL, rR).min())
        if ok_wall >= 0.15 and kmax <= 1.7:
            final, kind = rl, '최적 레이싱 라인'
            break
        why = f'검사 불통과 (벽까지 최소 {ok_wall:.2f} m, 최대 곡률 {kmax:.2f})'
        log(f'  {why}')
    if kind != '최적 레이싱 라인':
        log(f'최적화 결과 사용 안 함 → 중앙선으로 대체')

    np.savetxt(OUT_CSV, np.column_stack([final, np.full(len(final), 0.3)]), fmt='%.4f', delimiter=',')
    _preview(grid, centerline, final)
    k = curvature(final)
    log(f'저장: {OUT_CSV}  [{kind}] {len(final)}점, 최소 회전반경 {1 / max(k.max(), 1e-6):.2f} m')
    return kind


def _preview(grid, center, final):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    img = np.where(grid.occ, 40, np.where(grid.free, 255, 170)).astype(np.uint8)[::-1]
    sc = 4
    im = Image.fromarray(img).convert('RGB').resize((grid.w * sc, grid.h * sc), Image.NEAREST)
    d = ImageDraw.Draw(im)

    def px(p):
        return ((p[0] - grid.ox) / grid.res * sc, (grid.h - (p[1] - grid.oy) / grid.res) * sc)
    for pts, col in ((center, (120, 120, 255)), (final, (220, 0, 0))):
        q = [px(p) for p in pts]
        d.line(q + [q[0]], fill=col, width=2)
    im.save(OUT_PNG)


# ---------------------------------------------------------------------- 노드
class LapRecorder(Node):
    def __init__(self):
        super().__init__('build_raceline')    # 실물은 시스템 시계. 시뮬에서 테스트할 때만 -p use_sim_time:=true
        self.min_lap = self.declare_parameter('min_lap_length', 10.0).value
        self.close_r = self.declare_parameter('close_radius', 1.0).value   # 출발점에서 이 거리 안으로 오면 한 바퀴
        self.near_r = self.declare_parameter('near_radius', 2.0).value     # 이 안을 스쳐 지나가거나 멈춰도 한 바퀴
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.VOLATILE))
        self.pts, self.travel, self.next_log = [], 0.0, 5.0
        self.map_msg, self.closed_at, self.done = None, None, False
        self.best_d = float('inf')            # 출발점 근처(near_r 안)에서 가장 가까웠던 거리
        self.last_move = None                 # 마지막으로 5 cm 이상 움직인 시각
        self.create_timer(0.1, self._tick)
        self.get_logger().info('기록 시작. teleop으로 트랙을 한 바퀴 돌아 출발점으로 돌아오세요.')

    def _on_map(self, msg):
        self.map_msg = msg

    def _tick(self):
        if self.done:
            return
        if self.closed_at is not None:
            # 한 바퀴 끝 → 마지막 구간이 반영된 지도를 잠깐 기다렸다가 생성
            fresh = self.map_msg is not None and \
                Time.from_msg(self.map_msg.header.stamp) > self.closed_at
            waited = (self.get_clock().now() - self.closed_at) > Duration(seconds=6.0)
            if (fresh or waited) and self.map_msg is not None:
                self.finish()
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.0))
        except Exception:     # noqa: BLE001
            self.get_logger().info('map→base_link 대기 중 (SLAM 켜져 있나요?)', throttle_duration_sec=3.0)
            return
        p = np.array([t.transform.translation.x, t.transform.translation.y])
        now = self.get_clock().now()
        if self.pts and np.hypot(*(p - self.pts[-1])) < 0.05:
            # 출발점 근처에서 3초 이상 멈춰 있으면 한 바퀴 끝으로 본다
            if (self.travel > self.min_lap and self.last_move is not None
                    and np.hypot(*(p - self.pts[0])) < self.near_r
                    and now - self.last_move > Duration(seconds=3.0)):
                self._close('출발점 근처에서 멈춤')
            return
        self.last_move = now
        if self.pts:
            self.travel += float(np.hypot(*(p - self.pts[-1])))
        self.pts.append(p)
        if self.travel >= self.next_log:
            self.get_logger().info(f'기록 중: {self.travel:.0f} m')
            self.next_log += 5.0
        if self.travel <= self.min_lap:
            return
        d = float(np.hypot(*(p - self.pts[0])))
        if d < self.close_r:
            self._close(f'출발점 {self.close_r:.1f} m 안')
        elif d < self.near_r:
            self.best_d = min(self.best_d, d)
            if d > self.best_d + 0.3:
                self._close(f'출발점 옆 {self.best_d:.1f} m를 지나침')
        elif self.best_d < self.near_r:      # 출발점 근처를 지나서 벗어남
            self._close(f'출발점 옆 {self.best_d:.1f} m를 지나침')

    def _close(self, why):
        if self.closed_at is not None:
            return
        self.get_logger().info(f'한 바퀴 완료 ({self.travel:.1f} m, {why}). 지도 갱신 기다리는 중... (teleop은 멈추세요)')
        self.closed_at = self.get_clock().now()

    def _one_lap(self):
        """기록이 한 바퀴를 넘었으면(출발점을 지나쳐 더 돈 경우) 첫 한 바퀴만 남긴다.
        min_lap 이상 달린 뒤 출발점에 가장 가까웠던 지점(2 m 안)에서 자른다."""
        pts = np.array(self.pts)
        seg = np.hypot(*np.diff(pts, axis=0).T)
        travel = np.concatenate([[0.0], np.cumsum(seg)])
        dist = np.hypot(pts[:, 0] - pts[0, 0], pts[:, 1] - pts[0, 1])
        cand = np.nonzero((travel > self.min_lap) & (dist < self.near_r))[0]
        if not len(cand):
            return pts
        # 출발점 근처로 처음 들어온 구간 안에서 가장 가까운 점
        first = cand[0]
        end = first
        while end + 1 < len(pts) and dist[end + 1] < self.near_r:
            end += 1
        cut = first + int(np.argmin(dist[first:end + 1]))
        if cut < len(pts) - 1:
            self.get_logger().info(f'한 바퀴 넘게 기록됨 → 첫 한 바퀴({travel[cut]:.1f} m)만 사용')
        return pts[:cut + 1]

    def finish(self):
        if self.done:
            return
        self.done = True
        if self.travel < self.min_lap or self.map_msg is None:
            self.get_logger().error(f'기록이 부족해요 ({self.travel:.1f} m, 지도 {"있음" if self.map_msg else "없음"}).')
            return
        grid = Grid(self.map_msg)
        kind = build(self._one_lap(), grid, self.get_logger().info)
        self._save_grid(grid)
        self._save_map()
        self.get_logger().info(f'완료 [{kind}]. 이제 실행: python3 {os.path.join(HERE, "raceline_pursuit_real.py")}')

    def _save_grid(self, grid):
        """장애물 구분용 벽 지도: 이 지도에 있던 벽 근처 라이다 점은 주행 때 장애물이 아니라 벽으로 본다.
        그래서 매핑할 때는 트랙에 다른 리모나 사람이 없어야 한다."""
        path = os.path.join(os.path.dirname(MAP_BASE), 'q1_grid.npz')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        np.savez_compressed(path, occ=grid.occ, free=grid.free, res=grid.res, ox=grid.ox, oy=grid.oy)
        self.get_logger().info(f'벽 지도 저장: {path}')

    def _save_map(self):
        """로컬라이제이션(Q2)용으로 slam_toolbox 포즈그래프(.posegraph/.data)를 저장한다."""
        os.makedirs(os.path.dirname(MAP_BASE), exist_ok=True)
        try:
            r = subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                                'slam_toolbox/srv/SerializePoseGraph', f"{{filename: '{MAP_BASE}'}}"],
                               capture_output=True, text=True, timeout=30)
            if os.path.exists(MAP_BASE + '.posegraph'):
                self.get_logger().info(f'지도 저장 완료: {MAP_BASE}.posegraph / .data  '
                                       '(mapper_params_localization.yaml의 map_file_name에 이 경로를 적으세요)')
            else:
                self.get_logger().error(f'지도 저장 실패: {(r.stdout + r.stderr).strip()[-150:]}')
        except Exception as e:     # noqa: BLE001
            self.get_logger().error(f'지도 저장 실패: {e}')


def main():
    rclpy.init()
    node = LapRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C → 지금까지 기록으로 생성 시도')
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
