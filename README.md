# Advanced Line Tracker for TurtleBot (ROS Noetic)

터틀봇을 처음 접하는 초보자도 0부터 스스로 워크스페이스를 만들고 로봇을 굴려볼 수 있도록 작성된 **'고급 라인 트레이싱(차선 유지 주행)'** 가이드 및 소스 코드입니다.

---

## 📖 작동 원리 (How it Works)

로봇(터틀봇)에서 무거운 영상 처리(OpenCV)를 직접 하면 라즈베리파이의 한계 때문에 화면이 뚝뚝 끊기고 로봇이 탈선하게 됩니다. 이를 해결하기 위해 **로봇과 PC의 역할을 완벽하게 분리**했습니다.

1. **로봇(TurtleBot)의 역할 (눈과 다리)**: 
   - 카메라로 앞을 찍어서 **압축 이미지(Compressed Image)** 형태로 가볍고 빠르게 Wi-Fi를 통해 PC로 쏴줍니다. (송신)
   - 모터 제어기는 대기하고 있다가 PC에서 조향 명령(`/cmd_vel`)이 오면 그대로 바퀴를 굴립니다. (수신)
2. **PC의 역할 (두뇌)**: 
   - 터틀봇이 보낸 압축 이미지를 받아서 고성능 CPU로 0.01초 만에 차선을 분석해 냅니다. (수신 및 연산)
   - 분석 결과를 바탕으로 "앞으로 가면서 왼쪽으로 꺾어!" 라는 명령(`/cmd_vel`)을 터틀봇으로 보냅니다. (송신)

---

## 🛠 처음부터 따라 하는 빌드 가이드 (Beginner's Guide)

### [1단계] 터틀봇(로봇) 세팅하기
카메라 영상을 딜레이 없이 빠르게 전송하려면 터틀봇에 압축 패키지를 설치해야 합니다. 터틀봇에 SSH로 접속한 뒤 아래 명령어를 입력합니다.
```bash
# 터틀봇 SSH 터미널에서 실행
sudo apt-get update
sudo apt-get install ros-noetic-compressed-image-transport
```

### [2단계] PC 세팅 (워크스페이스 및 패키지 만들기)
PC 터미널을 열고, 로봇 코드를 담을 작업 공간(Workspace)을 처음부터 만들어 봅니다.

**1. 워크스페이스(Workspace) 만들기**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/
catkin_make
```

**2. 패키지(Package) 생성하기**
파이썬(rospy)과 OpenCV(cv_bridge)를 사용하는 패키지를 생성합니다.
```bash
cd ~/robot_ws/src
catkin_create_pkg color_pkg rospy cv_bridge sensor_msgs geometry_msgs
```

**3. 소스 코드 넣고 실행 권한 주기**
깃허브에 있는 `line.py`와 `line1.py` 파일을 다운받아 아래 경로에 넣습니다.
- 파일 위치: `~/robot_ws/src/color_pkg/scripts/` (scripts 폴더가 없으면 만드세요)
```bash
mkdir -p ~/robot_ws/src/color_pkg/scripts/
cd ~/robot_ws/src/color_pkg/scripts/
# (이곳에 line.py, line1.py 파일을 복사해서 넣으세요)

# 리눅스에서는 파이썬 파일에 실행 권한을 주어야 실행됩니다.
chmod +x line.py line1.py
```

**4. 최종 빌드하기**
```bash
cd ~/robot_ws/
catkin_make
source devel/setup.bash
```

---

## 🚀 실행 방법 (How to Run)

모든 준비가 끝났습니다! 로봇과 PC를 같은 Wi-Fi에 연결하고 아래 순서대로 실행하세요.

**1. 터틀봇 단말기 (SSH 접속)**
```bash
# 터미널 1 (로봇 센서 및 모터 활성화)
roslaunch aicon_bringup aicon_robot.launch

# 터미널 2 (새 창을 열고 카메라 활성화)
roslaunch aicon_bringup aicon_camera.launch
```

**2. PC 단말기**
```bash
# 라인 트레이싱 코드 실행
rosrun color_pkg line1.py
```

---

## 💻 노드(Node) 상세 설명 및 조작법

### 1. `line1.py` (최적화된 고급 알고리즘 - **추천**)
가장 빠르고 안정적인 주행을 위한 고급 라인 트레이싱 노드입니다.

- **바닥 노이즈 무시 (Contour Split)**: 화면을 '왼쪽 절반'과 '오른쪽 절반'으로 나눈 뒤, 각 절반에서 **화면 중앙과 가장 가까운 선 딱 1개씩만 선택**합니다. 이 방식을 통해 화면 구석에 보이는 거대한 하얀색 바닥(노이즈)을 완벽하게 무시합니다.
- **오실레이션 방지 (PD 제어)**: 카메라 딜레이로 인해 로봇이 비틀거리는 현상을 막기 위해 미분 제어(D-control)를 적용하여 조향을 부드럽게 만들었습니다.

**[실시간 키보드 조작 메뉴얼]**
실행 중인 터미널 창에서 프로그램이 켜져 있을 때 아래 키를 누르면 즉시 세팅이 바뀝니다.
- `[W] / [S]` : 직진 속도 증가 / 감소
- `[A] / [D]` : 회전 감도(Gain) 증가 / 감소 (로봇이 심하게 흔들리면 D를 눌러 감도를 낮추세요)
- `[I] / [K]` : 스캔 영역 위아래로 조절 (멀리 보려면 I, 발밑을 보려면 K)
- `[J] / [L]` : 스캔 영역 좌우 폭 좁게/넓게 (차선 밖의 바닥이 자꾸 인식되면 J를 눌러 폭을 좁히세요)
- `[Q]` : 프로그램 종료

### 2. `line.py` (기본형 / 상태 머신형)
차선을 잃어버렸을 때 후진하거나 제자리 회전하는 등의 '상태 머신(State Machine)' 로직이 적용된 기본형 노드입니다.
- **특징**: 화면 전체의 흰색 픽셀 중심점(Centroid)을 구하여 주행합니다. 알고리즘이 아주 단순하여 기초 원리 학습 및 코드 분석용으로 좋습니다.
