# Indoor Autonomous Transport Robot


---

## Cora-z7


by StolenHat, mar1794

---

## Jetson


by moka

---

## RPi 5
### 1. Sensor Fusion (LiDAR + Ultrasonic)
- Problem: LiDAR가 감지하지 못하는 유리문이나 난간 등의 장애물 인식 한계.
- Solution: merge_lidar_echo_node를 개발하여 0°, 90°, 180°, 270° 방향의 초음파 데이터를 LiDAR 스캔 데이터와 병합합니다.
- Result: 난간을 통과하려던 기존 경로와 달리, 장애물을 안정적으로 회피하는 경로를 생성합니다.

### 2. Autonomous Navigation (Nav2)
- Planner: NavfnPlanner 및 A* 알고리즘을 사용한 전역 경로 생성.
- Controller: MPPI Controller를 적용하여 장애물 거리와 목표 위치를 고려한 최적 속도 명령 발행.
- Costmap: Static, Obstacle, Inflation Layer를 구성하여 실시간 충돌 회피 수행.

### 3. User Tracking Mode
- Jetson으로부터 UDP 통신을 통해 수신한 사용자의 거리와 각도 데이터를 기반으로 실시간 목표 좌표(goal_pose)를 생성하여 추종 주행을 수행합니다

### Software Implementation Details
> ROS 2 Node Structure
Raspberry Pi 5에서 실행되는 주요 개발 노드는 다음과 같습니다:
- zynq_node: Cora Z7과 UART 통신을 통해 속도 명령 전달 및 엔코더/초음파 데이터 수집.
- merge_lidar_ultrasonic_node: LiDAR와 초음파 센서 데이터를 병합하여 /scan_merged 토픽 발행.
- person_receiver: Jetson으로부터 사용자 위치 데이터를 수신하여 Nav2 목표 지점 변환.
- mode_gui: Tkinter 기반의 사용자 모드(자율주행/추적) 선택 인터페이스.

> Special Configuration: sllidar_ros2
> 본 프로젝트에서는 LiDAR 드라이버인 sllidar_ros2 패키지를 커스텀하여 사용했습니다.
> Modification: view_sllidar_a1_launch.py 파일 내의 출력 토픽명을 센서 퓨전 노드와의 호환을 위해 **scan_merged**로 수정하여 적용하였습니다.

### 결과물
- GUI: 목적지(호실) 선택 및 로봇 상태 실시간 모니터링.
- Visualization: RViz2를 통한 맵, 경로, 센서 데이터 시각화

by Su
