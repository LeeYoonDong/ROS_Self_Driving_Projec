# SLAM 기반 자율주행 알고리즘

이 프로젝트는 SLAM(Simultaneous Localization and Mapping) 알고리즘을 기반으로 한 자율주행 시스템입니다. 2D LiDAR 센서를 사용하여 주변 환경을 인식하고, 장애물을 피해 자율적으로 주행하는 알고리즘을 구현했습니다.

## 주요 기능

1. 2D LiDAR 데이터 처리: 주변 환경의 거리 정보를 실시간으로 수집 및 처리합니다.
2. 장애물 감지: 수집된 데이터를 기반으로 주변 장애물을 감지합니다.
3. 경로 계획: Follow the Gap Method (FGM)를 사용하여 최적의 주행 경로를 결정합니다.
4. 자율 주행: 결정된 경로를 따라 자동으로 주행합니다.

## 사용된 기술

- Python
- ROS (Robot Operating System)
- 2D LiDAR 센서
- Follow the Gap Method (FGM)

## 시스템 요구사항

- ROS Kinetic 이상
- Python 2.7 또는 Python 3.x
- 2D LiDAR 센서 (예: RPLIDAR A1)

## 설치 방법

1. ROS 워크스페이스에 이 프로젝트를 클론합니다:
   ```
   cd ~/catkin_ws/src
   git clone [your-repository-url]
   ```

2. 의존성 패키지를 설치합니다:
   ```
   sudo apt-get install ros-[your-ros-distro]-rplidar-ros
   ```

3. 워크스페이스를 빌드합니다:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

## 사용 방법

1. ROS 마스터를 실행합니다:
   ```
   roscore
   ```

2. 2D LiDAR 노드를 실행합니다 (RPLIDAR A1 사용 예시):
   ```
   roslaunch rplidar_ros rplidar.launch
   ```

3. 자율주행 노드를 실행합니다:
   ```
   rosrun [your-package-name] lidar_auto_drive_1.py
   ```

## 파일 설명

- `lidar_auto_drive_1.py`: 기본 자율주행 알고리즘 (속도: 4500)
- `lidar_auto_drive_2.py`: 고속 자율주행 알고리즘 (속도: 14500)

## 알고리즘 설명

이 프로젝트는 Follow the Gap Method (FGM)를 사용하여 자율주행을 구현합니다:

1. LiDAR 데이터를 세 영역(좌, 우, 중앙)으로 나눕니다.
2. 각 영역에서 장애물이 없는 최대 간격(gap)을 찾습니다.
3. 가장 큰 간격이 있는 방향으로 조향합니다.
4. 일정한 속도로 전진합니다.

## 주의사항

- 이 알고리즘은 시뮬레이션 환경에서 테스트되었습니다. 실제 환경에서 사용할 경우 추가적인 안전 조치가 필요할 수 있습니다.
- 고속 버전(`lidar_auto_drive_2.py`)을 사용할 때는 특히 주의가 필요합니다.


## 라이선스

이 프로젝트는 [MIT 라이선스](https://opensource.org/licenses/MIT)에 따라 라이선스가 부여됩니다.

<br/>
<br/>

# SLAM-based Autonomous Driving Algorithm

This project is an autonomous driving system based on the SLAM (Simultaneous Localization and Mapping) algorithm. It uses a 2D LiDAR sensor to perceive the surrounding environment and implements an algorithm to autonomously navigate while avoiding obstacles.

## Key Features
1. 2D LiDAR Data Processing: Collects and processes real-time distance information of the surrounding environment.
2. Obstacle Detection: Detects surrounding obstacles based on the collected data.
3. Path Planning: Determines the optimal driving path using the Follow the Gap Method (FGM).
4. Autonomous Driving: Automatically drives along the determined path.

## Technologies Used
- Python
- ROS (Robot Operating System)
- 2D LiDAR sensor
- Follow the Gap Method (FGM)

## System Requirements
- ROS Kinetic or higher
- Python 2.7 or Python 3.x
- 2D LiDAR sensor (e.g., RPLIDAR A1)

## Installation
1. Clone this project into your ROS workspace:
   ```
   cd ~/catkin_ws/src
   git clone [your-repository-url]
   ```
2. Install dependency packages:
   ```
   sudo apt-get install ros-[your-ros-distro]-rplidar-ros
   ```
3. Build the workspace:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

## Usage
1. Run the ROS master:
   ```
   roscore
   ```
2. Run the 2D LiDAR node (example using RPLIDAR A1):
   ```
   roslaunch rplidar_ros rplidar.launch
   ```
3. Run the autonomous driving node:
   ```
   rosrun [your-package-name] lidar_auto_drive_1.py
   ```

## File Description
- `lidar_auto_drive_1.py`: Basic autonomous driving algorithm (speed: 4500)
- `lidar_auto_drive_2.py`: High-speed autonomous driving algorithm (speed: 14500)

## Algorithm Explanation
This project implements autonomous driving using the Follow the Gap Method (FGM):
1. Divides LiDAR data into three regions (left, right, center).
2. Finds the maximum gap without obstacles in each region.
3. Steers towards the direction with the largest gap.
4. Moves forward at a constant speed.

## Precautions
- This algorithm has been tested in a simulation environment. Additional safety measures may be necessary when used in a real environment.
- Extra caution is required when using the high-speed version (`lidar_auto_drive_2.py`).

## License
This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).



