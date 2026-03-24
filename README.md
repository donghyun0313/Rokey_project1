
# Dishwasher Robot System (DSR-01)

Cloud-Connected Intelligent Robotic Dishwashing Solution

이 프로젝트는 Doosan Robotics M0609 협동 로봇과 Firebase 실시간 데이터베이스를 결합하여, 식판의 수거부터 세척, 적재까지의 전 과정을 자동화한 스마트 팩토리 솔루션입니다. 웹 대시보드를 통해 실시간으로 로봇을 제어하고 공정 상태를 모니터링할 수 있습니다.

## 🚀 Key Features

- Firebase 클라우드 기반의 실시간 원격 제어 -> Firebase RTDB 연동을 통해 웹 UI 로 로봇 제어 가능 (Start, Pause, Resume)
- ROS2의 멀티스레딩이 결합된 고수준의 로봇 자동화 시스템


- Safety Checkpoint System: 모든 공정 단계에 체크포인트를 삽입하여 사용자의 중단/일시정지 명령에 즉각적으로 반응하는 런타임 제어 로직.

- Compliance Control: 식판 파지 시 물리적 충격을 완화하기 위해 로봇의 강성을 조절하는 힘 제어(Compliance) 기술 적용.

- Advanced Cleaning Patterns: 단순 동작이 아닌 지그재그 패턴 및 J6 축 회전을 결합한 정밀 솔질 세척 알고리즘.

- Multi-threaded Execution: ROS2 Spin, Firebase Listener, Main Logic을 분리하여 끊김 없는 명령 수신 및 상태 보고.


## 🛠️ System Architecture
본 시스템은 클라우드 계층, 로직 제어 계층, 물리 실행 계층으로 구성됩니다.



### 🍟System Workflow

![System Workflow](/pictures/proj1.png)


## 📋 Full Operation Flow
DR-M0609 로봇은 총 6단계의 정밀 공정을 수행하며, 모든 상태는 Firebase를 통해 실시간 업데이트됩니다.

1. **PICK (수거)**: Compliance 제어를 통해 식판을 부드럽게 감지하고 파지.

2. **TURBID (잔반 처리)**: 고속 진동(Shake) 모션을 통해 식판의 음식물을 효과적으로 제거.

3. **HOT (온수 세척)**: 주기적 모션(Periodic Motion)을 활용해 온수 탱크에서 애벌 세척을 수행.

4. **BRUSH (솔질 세척)**: 사용자 좌표계를 기준으로 정교한 지그재그 패턴의 솔질을 수행.

5. **COLD (냉수 헹굼)**: 깨끗한 물로 식판을 최종 헹굼 처리.

6. **STACK (적재)**: 세척된 식판을 지정된 스택 위치에 정교하게 쌓음.

---

## 💻 Tech Stack & Requirements

### 🌿 Environments
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **프레임워크**: ROS 2 Humble Hawksbill
- **프로그래밍 언어**: Python 3.10+

### 🤖 Robotics Middleware & Libraries
- **Doosan Robot ROS2 API**
  - 패키지 : `dsr_msgs2`
  - 사용되는 두산 협동 로봇 모델 : M0609
- **libraries:**
  - rclpy
  - dsr_msgs2
  - DSR_ROBOT2
  
<br>

## `requirements.txt` 외 의존성 파일들 다운로드 하기

```
# ROS2
sudo apt install ros-humble-desktop

# Doosan ROS2 API
sudo apt install ros-humble-dsr-msgs2

# Python deps
pip install -r requirements.txt
```


<br>

## ⚙️ Installation & Setup

### Set up Environment
1️⃣ ROS 2 Humble 다운로드
```
sudo apt update
sudo apt install ros-humble-desktop
```

2️⃣ ROS 2 워크스페이스 생성
```
mkdir -p ~/cobot_ws/src
cd ~/cobot_ws
colcon build
source install/setup.bash

```

3️⃣ Install Doosan Robot ROS2 Packages
```
sudo apt install ros-humble-dsr-msgs2
```

---

### How to Run
1️⃣ Source ROS 2 
```
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
```

2️⃣ 로봇 드라이버 가동

- <u>real 로봇과 연결할 때,</u>
  - `ros2 launch dsr_launcher2 dsr_launcher2.launch.py mode:=real model:=m0609`

- <u> simulator 로봇 시뮬레이션과 연결할 때, </u>
  - `ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609`


3️⃣ 런치 로봇 컨트롤 시스템
```
ros2 run cobot1 dishwasher_real
```


4️⃣ ros bridge server 와 연결하기


```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=7070
```

#### **예시 사진** 

![System2](/pictures/run_commands.png)

<br>

## 🎮 서비스콜 컨트롤 명령어
웹 UI 대시보드 외에 터미널(CLI)에서도 로봇 제어가 가능합니다.


**공정 시작 (START)**
`ros2 service call /dsr01/dishwasher/start std_srvs/srv/Trigger`

**일시 정지 (PAUSE)**
`ros2 service call /dsr01/dishwasher/pause std_srvs/srv/Trigger`

**재개 (RESUME)**
`ros2 service call /dsr01/dishwasher/resume std_srvs/srv/Trigger`



## ⚠️ Safety & Error Handling (안전 복구 모드)
- Runtime Checkpoint: 로봇 동작 중 stop_requested 플래그가 감지되면 즉시 동작을 멈추고 안전 모드로 전환합니다.

- Real-time Reporting: 예외 상황 발생 시 에러 메시지를 Firebase에 즉시 업로드합니다.
