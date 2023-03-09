# turtlebot_ws

HOW TO GUIDE
---------------MAP---------------
--SAVE MAP--
Open rviz when robot is in either (a)gazebo or (b)physical environment and let the robot roam to explore the map
>ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/install/map_loader/share/map_loader/launch/map
This saves the map into the path defined

--LAUNCHING SAVED MAP--
Configuration file for map can be found in src/map_loader/launch/load_map.launch.py
terminal 1: >grslam
terminal 2: >ros2 launch src/map_loader/launch/load_map.launch.py

--TROUBLESHOOTING--
no map received: restart both apps
