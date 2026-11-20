# 실물 LIMO용 코드 (팀6)

시뮬 코드(`ws_limo_humble/src/limo_ros2/limo_slam/scripts/`)는 그대로 두고, 실물용은 이 폴더에 복사본으로 따로 만들었습니다.

| 파일 | 시뮬 원본 | 바꾼 점 |
| --- | --- | --- |
| `raceline_pursuit_real.py` | `raceline_pursuit.py` | 출력 `/limo/ack_cmd`(AckermannDrive), `use_sim_time` 끔, 라이다 토픽·경로 파일을 옵션으로, 실물 속도·축거(0.20 m), 라이다가 끊기면 정지, **[A] 지도 기반 장애물 구분, [D] 속도 결정 정리** (아래 참고) |
| `build_raceline_real.py` | `build_raceline.py` | 경로를 이 폴더에 저장(`q1_raceline_real.csv`), 최적화 도구가 없으면 중앙선으로 저장, 로컬라이제이션용 지도 저장(`maps/q1_map`), 장애물 구분용 벽 지도 저장(`maps/q1_grid.npz`), 출발점 근처 통과/정지도 한 바퀴로 판정, `use_sim_time` 끔 |
| `mapper_params_online_async.yaml` | 같은 이름 | `use_sim_time: false` |
| `mapper_params_localization.yaml` | 같은 이름 | `use_sim_time: false`, 지도 경로와 시작 자리는 아래 참고 |

## 추가 알고리즘 (2026-11-20, 시뮬 검증)

- **[A] 지도 기반 장애물 구분**: 매핑 때 저장한 벽 지도(`maps/q1_grid.npz`)에서 벽 근처(12 cm)의 라이다 점은 벽, 나머지는 장애물로 본다.
  충돌 검사와 경로 비용은 벽+장애물 모두 보고, **장애물 옆 감속에만 장애물 점을 쓴다** (벽 옆에서 감속하지 않게). 매핑할 때 트랙이 비어 있어야 한다. 끄기: `-p use_wall_map:=false`
- **[D] 속도 결정 정리**: v = min(레이싱 라인 속도, local path 곡률 제한, 장애물 옆 간격 제한(그 지점까지 감속 가능한 속도), 조향 횡가속도 제한). 가속만 `a_long_max`로 제한.
  예전의 "레이싱 라인에 장애물이 보이면 무조건 감속", "핸들 각도 비례 감속"은 뺐다.
- 새 파라미터: `v_gap_min`(장애물 바로 옆 속도, 0.15), `gap_full`(이 간격 이상이면 장애물 감속 없음, 0.30 m), `a_brake`(0.4), `wall_tol`(0.12 m), `use_wall_map`(true)
- 시뮬 비교 (맵마다 2회, v_max 1.0): 장애물 없음 48.6→46.9 s, 장애물1 59.3→51.8 s, 장애물3 74.9→55.7 s (평균). 장애물과 차체 최소 간격 0 cm(스침)→2.7~9.4 cm. 제어 계산 평균 5~8 ms / 최대 35 ms (주기 50 ms)

## 실물에서 처음 확인할 것 (순서대로)

터미널마다 `export ROS_DOMAIN_ID=6`, `export ROS_LOCALHOST_ONLY=0` 을 먼저 입력합니다. (조교님 번호 확인 후 변경)

1. bringup (기말실습공지 방식): 도커 `yspark98/limo1_humble` 컨테이너 안에서 `ros2 launch wego teleop_launch.py`
   - tf, 라이다 `/scan`, `/imu`, `/odom`, EKF `/odom/filtered`가 한 번에 켜집니다. LIMO1(Jetson Nano)은 카메라를 끄는 게 좋습니다.
   - 패키지 설치는 컨테이너 안에서만, 코드는 `~/Documents` 안에만 둡니다.
2. 명령 토픽 확인: `ros2 topic list`에서 `/cmd_vel`(Twist)인지 `/limo/ack_cmd`(AckermannDrive)인지 봅니다.
   - `/cmd_vel`이면 주행할 때 `-p output:=twist`, `/limo/ack_cmd`면 기본값 그대로.
3. 토픽 확인: `ros2 topic list` → `/scan`(또는 다른 이름), `/odom`, `/tf`, `/limo/ack_cmd`
   - 라이다 토픽 이름이 `/scan`이 아니면 `mapper_params_*.yaml`의 `scan_topic`과 주행 옵션 `-p scan_topic:=이름`을 바꿉니다.
4. TF 확인: `ros2 run tf2_ros tf2_echo odom base_link` (나오면 정상), `ros2 run tf2_ros tf2_echo base_link <라이다 프레임>`
   - 라이다 프레임이 base_link와 연결돼 있지 않으면 슬램이 안 됩니다. 정적 TF를 추가해야 합니다.
5. 명령 방향 확인 (바퀴를 띄우거나 넓은 곳에서, 주변을 비우고):
   ```bash
   ros2 topic pub --once /limo/ack_cmd ackermann_msgs/msg/AckermannDrive "{speed: 0.1, steering_angle: 0.2}"
   ```
   앞으로 가고 **왼쪽**으로 꺾이면 정상(조향각 + = 왼쪽). 오른쪽이면 `steering_angle` 부호를 반대로 해야 합니다. 안 움직이면 단위나 모드를 확인합니다.

## 매핑 테스트 (Q1)

```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<이 폴더>/mapper_params_online_async.yaml
python3 <이 폴더>/build_raceline_real.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # 천천히 한 바퀴 돌아 출발점 근처로 돌아오기
```
- 10 m 이상 달린 뒤 출발점 1 m 안으로 들어오거나, 2 m 안을 스쳐 지나가거나, 2 m 안에서 3초 이상 멈추면 한 바퀴로 판정합니다. 한 바퀴를 넘게 돌았으면 첫 한 바퀴만 씁니다.
- `완료 [...]`가 나오면 `q1_raceline_real.csv`(경로)와 `maps/q1_map.posegraph/.data`(지도)가 이 폴더에 생깁니다.
- 지도가 찌그러지지 않았는지 `q1_raceline_real.png`로 확인합니다. (파란 선 = 중앙선, 빨간 선 = 최종 경로)
- 리모에는 최적화 도구(Raceline-Optimization)가 없을 가능성이 높아서 중앙선이 저장됩니다. 노트북 도커에서 같은 지도로 최적 라인을 만들 수도 있습니다.

## 로컬라이제이션 테스트 (Q2)

`mapper_params_localization.yaml`에서 두 줄을 실제 값으로 바꿉니다.
- `map_file_name`: 지도 저장 로그에 나온 경로(확장자 없이). 예: `/home/wego/Documents/team6/limo_team6/real/maps/q1_map` (LIMO1 컨테이너는 `/home/agilex/Documents/...`)
- `map_start_pose`: 지도를 만들기 시작한 자리. bringup 직후 그 자리에서 매핑을 시작했다면 `[0.0, 0.0, 0.0]`. **로컬라이제이션도 같은 자리, 같은 방향에서 bringup 후 시작해야 합니다.**

```bash
ros2 launch slam_toolbox localization_launch.py slam_params_file:=<이 폴더>/mapper_params_localization.yaml
```
RViz에서 로봇 위치가 지도 위 실제 위치와 맞는지 확인합니다. 로봇을 손으로 조금 옮겨 봐도 위치가 따라와야 합니다.

## 주행

```bash
python3 <이 폴더>/raceline_pursuit_real.py                              # 기본 v_max 0.4 m/s
python3 <이 폴더>/raceline_pursuit_real.py --ros-args -p v_max:=0.25    # 더 천천히
```
- 처음에는 장애물 없이 낮은 속도로, 주변을 비우고 시작합니다.
- 라이다 데이터가 0.5 s 이상 끊기면 멈춥니다. 시작 전에 라이다가 안 들어오면 `라이다 수신 없음 → 정지 대기`가 나옵니다.
- Ctrl+C로 멈추면 정지 명령(속도 0)을 내보냅니다.

## 아직 확인되지 않은 것 (실물 테스트 필요)
- 라이다 토픽 이름과 프레임, base_link와의 TF
- `steering_angle`의 단위(rad 가정)와 부호
- 리모 odom 품질 (바퀴 오도메트리 + IMU). 매핑이 찌그러지면 속도를 더 낮춥니다.
- 실물 최고 속도와 가감속, 코너 속도 (`v_max`, `a_lat_max`는 보수적으로 잡은 첫 값)
