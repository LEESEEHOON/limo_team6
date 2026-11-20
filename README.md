# 팀6 LIMO 사용 가이드

스마트모빌리티공학실험1 · 마지막 업데이트 2026-11-20

> **한 줄 요약**: 코드 수정과 push는 이시훈 노트북에서만 하고, 리모에서는 `git pull`만 한다.
> 실습 때 받은 리모가 **LIMO2 PRO인지 LIMO1인지** 먼저 확인한다. 구동 방법이 다르다.

## 현재 상태

| 항목 | 상태 |
|---|---|
| 리모 접속 (SSH) | 완료 (LIMO2 PRO) |
| 리모 구동 (bringup, Ackermann) | 완료 (LIMO2 PRO), `/odom` `/tf` `/imu` 확인 |
| 라이다 `/scan` | 미확인. 다음 실습 때 `wego teleop_launch.py`로 확인 |
| 조별 `ROS_DOMAIN_ID` | 조교님께 확인 필요 (임시로 6 사용, 겹치는 조 없음 확인) |
| 대회 후진 허용 여부 | 다음 수업 때 조교님께 질문 |
| 실물 매핑 (지도 생성/저장) | 예정 |
| 실물 위치추정 (저장한 지도) | 예정 |

## 1. 리모 종류

| | LIMO2 PRO | LIMO1 |
|---|---|---|
| 컴퓨터 | Jetson Orin Nano | Jetson Nano (느림 → 카메라는 끄는 게 좋다) |
| ROS | 리모 본체에 ROS 2 Humble 설치됨 | 도커 컨테이너 안에서 ROS 2 Humble |
| 구동 | `ros2 launch wego teleop_launch.py` (또는 아래 3-1) | 도커 컨테이너 만든 뒤 `ros2 launch wego teleop_launch.py` |
| 라이다 | | LIMO2 PRO와 다른 라이다 (스캔 범위, 점 개수, 주기가 다름) |

조교님이 두 리모의 사용 환경을 같게 맞추려고 LIMO1은 도커로 구성했다. 11/18 실습에서 쓴 것은 LIMO2 PRO다.

## 2. 리모 접속

```bash
ssh wego@172.30.1.38
```

- 처음 접속하면 `yes`를 입력한다.
- 환경 선택 메뉴가 나오면 `1`을 선택한다. **메뉴에서 Ctrl+C 금지.**
- IP는 와이파이가 바뀌면 달라진다. 리모 화면에서 `ip -4 -o a`로 확인한다.
- 접속이 안 되면 리모와 노트북이 같은 와이파이인지 먼저 확인한다.

## 3. ROS 환경 설정 (터미널을 열 때마다 입력)

```bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
```

- 터미널을 닫으면 사라진다. 매번 입력한다. 도커 컨테이너 안 터미널도 마찬가지다.
- **리모의 `~/.bashrc`에는 넣지 않는다.** 공용 계정이라 다른 조까지 바뀐다.
- 리모 기본값: `ROS_DISTRO=humble`, `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID` 없음(=0)
- 확인: `printenv | grep ROS`, `echo $ROS_DOMAIN_ID`
- bringup 전에 `ros2 topic list`에 `/parameter_events`, `/rosout`만 나오면 같은 번호를 쓰는 다른 조가 없는 것이다.

## 4. 리모 구동 (bringup)

켜기 전: 주변을 비우고, 다른 조가 bringup 중인지 확인한다. **bringup과 teleop은 한 번에 한 명만.** 끄기는 `Ctrl+C`.

### 4-1. LIMO2 PRO

공식 방법 (실습 공지):
```bash
ros2 launch wego teleop_launch.py
```

11/18에 쓴 방법 (라이다는 따로 켜야 한다):
```bash
ros2 launch limo_base limo_base_ackerman.launch.py
ros2 launch limo_base open_ydlidar_launch.py
```

- `start_limo.launch.py`는 전체가 주석 처리되어 아무 일도 안 하니 쓰지 않는다.
- launch 목록: `limo_base_ackerman` / `limo_base` / `limo_base_mecanum` / `open_ydlidar_launch`

### 4-2. LIMO1 (도커)

컨테이너 만들기 (리모마다 한 번, 이름은 팀 전용으로):
```bash
sudo docker run -it \
  --name limo_humble_team6 \
  --network host \
  --privileged \
  --device=/dev/ttyTHS1:/dev/ttyTHS1 \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/agilex/Documents:/home/agilex/Documents \
  yspark98/limo1_humble:latest \
  bash
```

실행:
```bash
sudo docker start limo_humble_team6
sudo docker exec -it limo_humble_team6 bash
cd /home/agilex/Documents/limo_basic_ws/
source install/setup.bash
ros2 launch wego teleop_launch.py
```

- 공지 예시의 이름 `limo_humble`을 그대로 쓰면 다른 팀 컨테이너와 겹칠 수 있다.
- 사용자 이름과 경로(`/home/agilex/...`)는 리모마다 다를 수 있으니 확인한다.

### 4-3. Ackermann 모드 (매뉴얼 슬라이드 20, 26)

| 앞 LED | 모드 |
|---|---|
| 녹색 | Ackermann |
| 황색 | 4륜 차동/트랙 |
| 청색 | 메카넘 |

- 차 전방 빨간 걸쇠를 올린 뒤 약 30도 돌려 고정하면 Ackermann이다.
- 모드를 바꾼 뒤에는 bringup을 껐다 다시 켠다.
- `Invalid motion mode ...` 경고가 나면 재실행한다. 우리는 재실행 후 사라졌다.

## 5. 토픽

공식 bringup(`wego teleop_launch.py`)의 토픽 (실습 공지):

| 토픽 | 내용 |
|---|---|
| `/scan` | 2D 라이다 |
| `/imu` | IMU |
| `/odom` | 오도메트리 |
| `/odom/filtered` | EKF로 보정한 오도메트리 |
| `/tf`, `/tf_static` | 좌표계 |
| `/camera/color/image_raw`, `/camera/color/camera_info` | RGB 카메라 |

11/18 LIMO2 PRO에서 `limo_base_ackerman.launch.py`만 켰을 때 확인한 토픽:

| 토픽 | 비고 |
|---|---|
| `/imu`, `/odom`, `/tf`, `/limo_status` | 정상 출력 |
| `/limo/ack_cmd` | 주행 명령. 타입 `ackermann_msgs/msg/AckermannDrive` (`steering_angle`, `speed`) |
| `/scan` | 없음 (라이다 launch를 따로 켜야 함) |

- 네임스페이스 `/limo`, 프레임 `odom` / `base_link`
- 로그의 `odom topic name: 0`은 이상해 보이지만 실제 `/odom`은 나온다.
- 공식 bringup에서 주행 명령 토픽이 `/cmd_vel`인지 `/limo/ack_cmd`인지는 아직 확인하지 않았다.
- 공지에 적힌 필수 토픽만 조교님이 확인했다. 그 밖의 토픽은 직접 확인한다.

## 6. 코드 받기 (팀원)

처음 한 번:
```bash
mkdir -p ~/Documents/team6 && cd ~/Documents/team6
git clone https://github.com/LEESEEHOON/limo_team6.git
```

이후에는 최신 코드만 받는다 (`clone`을 다시 하지 않는다):
```bash
cd ~/Documents/team6/limo_team6
git pull
```

- 저장소는 공개라 아이디와 토큰을 묻지 않는다. `Username`을 물으면 입력하지 말고 `Ctrl+C`.
- **리모에서 push 금지.** 토큰이 공용 리모에 남을 수 있다.
- 실습 공지: 코드는 `~/Documents` 안에만 둔다.

## 7. 도커 개념

| | 뜻 | 누가 만드나 | 바꿔도 되나 |
|---|---|---|---|
| 이미지 (`yspark98/limo1_humble:latest`) | 설치 원본 (틀) | 조교님 (Docker Hub에 있음) | 원본은 바꾸지 않는다 |
| 컨테이너 | 이미지로 만든 실행 환경 | 우리가 `docker run`으로 만든다 | 우리 것이라 패키지 설치 가능 |

- 컨테이너는 **만든 리모 안에만** 있다. 리모가 바뀌면 그 리모에서 다시 만들어야 한다.
- 실습 공지: 패키지 설치는 **반드시 도커 컨테이너 안에서만** 한다.
- 실습이 끝나면 조교님이 리모의 도커 이미지와 코드를 전부 지운다. 코드는 GitHub, 우리가 설치한 환경은 Dockerfile(설치 순서를 적은 파일)을 GitHub에 두거나 이미지를 Docker Hub에 올려 보관한다.
- 도커 명령은 `sudo`가 필요하다.
- 11/18 확인 결과 LIMO2 PRO는 도커 없이 리모 본체에서 ROS를 바로 실행한다 (`sudo docker images`, `sudo docker ps -a` 모두 비어 있었음). 도커가 필요한 건 LIMO1이다.

```bash
sudo docker images
sudo docker ps -a
```

## 8. 주의사항

**하드웨어 (실습 공지)**
- 전원 버튼을 세게 누르지 않는다. 버튼이 안쪽으로 밀려 들어가는 사례가 많다.
- 책상, 의자 등 높은 곳에 올려두지 않는다. 떨어져 파손되는 사례가 많다.
- 화면이 안 켜지면 디스플레이 연결부부터 확인한다. 연결부가 약해 잘 빠진다.
- 배터리는 지정된 충전기만 쓰고, 충전이 끝나면 바로 분리한다 (화재 예방).
- **충전 중에는 로봇 연결 절대 금지.**
- 고장이나 파손이 생기면 바로 조교님과 교수님께 알린다.

**공용 리모**
- 다른 조 컨테이너를 지우거나 멈추지 않는다.
- 조교님 허락 없이 리모 본체에 프로그램(tmux 등)을 설치하지 않는다.
- docker 그룹과 소켓 권한을 바꾸지 않는다.
- 토큰을 URL에 넣거나 저장하지 않는다.
- 로봇을 움직이기 전에 주변을 비운다. teleop은 천천히.
- 매핑할 때는 트랙에 다른 리모나 사람이 없게 한다 (지도에 벽으로 남는다).

## 9. 실습 운영과 대회

- 리모 7대, 10팀 (1분반 6팀, 2분반 4팀). 2대는 시험용이라 LIMO1 4대 + LIMO2 PRO 1대로 실습한다. LIMO2 PRO는 모든 팀이 최소 1회 쓴다.
- 시간: 18:00~20:00 5팀, 20:00~22:00 5팀. 주차마다 1분반/2분반 교대.
- F1 트랙은 A room에 설치된다.
- 대회는 실물로 한다. 다른 리모도 장애물이고, 멈추거나 뒤로 움직일 수 있다.
- 후진 허용 여부는 확인 필요.

## 10. 다음 할 일

- [ ] 조교님께 확인: 조별 `ROS_DOMAIN_ID`, 대회 후진 허용 여부
- [ ] 받은 리모 종류 확인 후 bringup, `ros2 topic list`로 `/scan`과 주행 명령 토픽 확인
- [ ] `ros2 run tf2_ros tf2_echo odom base_link`로 TF 확인
- [ ] teleop으로 천천히 움직여 보기
- [ ] 실물 매핑: 지도 생성과 저장
- [ ] 실물 위치추정: 저장한 지도에서 위치 확인
