# turtlebot_ws

<details><summary>Template</summary>
<p>

</p>
</details>

HOW TO GUIDE <br>
<details><summary>Map</summary>
<p>

<details><summary>Saving map</summary>
<p>
Open rviz when robot is in either (a)gazebo or (b)physical environment and let the robot roam to explore the map <br>
```python
   ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/install/map_loader/share/map_loader/launch/<map>
```
This saves the map into the path defined <br>
</p>
</details>

<details><summary>Launching saved map</summary>
<p>
Configuration file for map can be found in src/map_loader/launch/load_map.launch.py <br>
terminal 1:
```console
grslam
```
terminal 2:
```console
ros2 launch src/map_loader/launch/load_map.launch.py
```

</p>
</details>

</p>
</details>

---------------MAP--------------- <br>
--SAVE MAP-- <br>
Open rviz when robot is in either (a)gazebo or (b)physical environment and let the robot roam to explore the map <br>
>ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/install/map_loader/share/map_loader/launch/<map> <br>
This saves the map into the path defined <br>

--LAUNCHING SAVED MAP-- <br>
Configuration file for map can be found in src/map_loader/launch/load_map.launch.py <br>
terminal 1:
>grslam 
<br>
terminal 2: <br>
>ros2 launch src/map_loader/launch/load_map.launch.py <br>

--TROUBLESHOOTING-- <br>
no map received: restart both apps <br>


