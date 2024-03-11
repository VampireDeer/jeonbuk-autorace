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
- 터미널 3개에서 순서데로 각각 실행

1

    ros2 launch minibot_bringup bringup_robot.launch.py

2

    ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[400,256]"

3

    ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/map.yaml

![gazebo1](https://github.com/VampireDeer/minibot/assets/132260442/5f07e61f-a5c7-42b9-b615-a42cd0974593)

---
### 2-2. 가제보와 rviz2에 minibot과 map 띄우기

gazebo -> minibot and map
  
    ros2 launch my_robot_description launch_sim.launch.py 

rviz2 

    rviz2

- rviz2 Displays 설정\
Fixed Frame -> base_link\
RobotModel\
TF\
LaserScan\
Map\
Camera

![gazebo_map](https://github.com/VampireDeer/minibot/assets/132260442/412266f3-cea2-4ede-88c9-182a27f835e0)

---
### 2-3. Gazebo 와 rviz2로 map building하기

2-2를 실행한 후

map building

    ros2 launch my_robot_description map_building.launch.py

teleopkeyboard

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

teleop keyboard로 조종하여 map building 후 map 저장

    ros2 run nav2_map_server map_saver_cli -f map_name

![map_building](https://github.com/VampireDeer/minibot/assets/132260442/d243875f-d9ce-4a16-8e49-b1c69313000f)

---
### 2-4. Ros2 SLAM

gazebo 

    ros2 launch my_robot_description launch_sim.launch.py

navigation2

    ros2 launch my_robot_description bringup.launch.py use_sim_time:=true map:=./map_custom.yaml

launch

    ros2 launch my_robot_description display.launch.py

![slam](https://github.com/VampireDeer/minibot/assets/132260442/38a4fbc0-17dd-4756-b541-59aefa7c8e16)


