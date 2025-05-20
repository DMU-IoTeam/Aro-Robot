# Aro-Robot ROS 2 Workspace 🚀

Raspberry Pi와 WSL 2 간 WebSocket 브릿지 & V-SLAM 데모


| Host | OS / ROS | 비고 |
|------|----------|------|
| **Pi 4** | Ubuntu 24.04 • ROS 2 Jazzy | rosbridge_server 설치 |
| **WSL 2** | Ubuntu 22.04 • ROS 2 Humble | Python 3.10, colcon |

공통적으로 사용하는 패키지
```bash
# Pi
sudo apt install ros-jazzy-rosbridge-suite
# WSL
sudo apt install python3-venv python3-colcon-common-extensions
python3 -m pip install --user roslibpy
```

WSL에서 워크스페이스 빌드 후 Pi와 WSL 각각 실행하면 된다.
### WSL 워크스페이스 빌드
```bash
git clone https://github.com/DMU-IoTeam/Aro-Robot.git
cd Aro-Robot
python3 -m venv ros-ws-venv && source ros-ws-venv/bin/activate
export PYTHONNOUSERSITE=1
colcon build
source install/setup.bash
```
### 실행
#### Pi
```bash
ros2 run demo_nodes_cpp talker --ros-args -r chatter:=/test_string
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
#### WSL
```bash
ros2 launch aro_bridge aro_bridge.launch.py \
  pi_ip:=<Pi Tailscale IP>
# 확인
ros2 topic echo /remote_pi/test_string
```

