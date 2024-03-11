# jeonbuk-autorace

## 1. 프로젝트 요약
### 1-1. 설명
자율 이동 로봇(AMR, Autonomous Mobile Robots)으로 주행 가이드 코드
### 1-2. 개발환경
- Ubuntu 22.04 Desktop
- ROS2 Humble
## 2. 프로젝트 
---
### 2-1. 로봇 실행하기
- SSH로 로봇에 부착 되어있는 Raspberry Pi에 접속 후 각각 접속한 터미널 3개에서 순서데로 각각 실행

1 로봇 구동

    ros2 launch minibot_bringup bringup_robot.launch.py

2 Pi Camera On

    ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[400,256]"

3 로봇 네비게이션 구동(map 이름에 따라 맨 뒤 yaml 파일이 바뀔 수 있다)

    ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/map_final.yaml

![image](https://github.com/VampireDeer/jeonbuk-autorace/assets/132260442/8689ea7e-6653-47be-adc7-1abc01619154)

---
### 2-2. rviz2 실행하기

1 SSH 접속이 아닌 메인 컴퓨터에서 rviz2 실행하기 (src 파일이 있는 곳에서 ros2 humble 실행, source ./install/localsetup.bash 후 실행 )
  
    rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz 

![image](https://github.com/VampireDeer/jeonbuk-autorace/assets/132260442/8689ea7e-6653-47be-adc7-1abc01619154)

---
### 2-3. Detect, Control Package launch 파일 구동

- SSH 접속이 아닌 메인 컴퓨터에서 터미널을 열어서 실행( ros2 humble 실행, source ./install/localsetup.bash 후 실행 )

1 ArUCo Marker Detect, Object Detect 

    ros2 launch minibot_detect detect_all.launch.py

![image (1)](https://github.com/VampireDeer/jeonbuk-autorace/assets/132260442/acc227c0-bf41-4a0c-a9ca-be6b41e9547c)

2 Lane Detect, Control ans SLAM

    ros2 launch minibot_driving run.launch.py

![image (2)](https://github.com/VampireDeer/jeonbuk-autorace/assets/132260442/41d686ed-cad9-4727-8b5c-982e42dd832a)

