# turtlebot_ws

HOW TO GUIDE <br>
---------------MAP--------------- <br>
--SAVE MAP-- <br>
Open rviz when robot is in either (a)gazebo or (b)physical environment and let the robot roam to explore the map <br>
>ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/install/map_loader/share/map_loader/launch/map <br>
This saves the map into the path defined <br>

--LAUNCHING SAVED MAP-- <br>
Configuration file for map can be found in src/map_loader/launch/load_map.launch.py <br>
terminal 1: >grslam <br>
terminal 2: >ros2 launch src/map_loader/launch/load_map.launch.py <br>

--TROUBLESHOOTING-- <br>
no map received: restart both apps <br>
