# turtlebot_ws

HOW TO GUIDE

<details><summary>Troubleshooting</summary>
<p>
  
- **No map received** - restart rslam and rosbu, make sure that you rosbu first then rslam
- **Connection Refused** - check same hotspot(NOT the nus), restart hotspot
- **RSLAM not working well** - Re-rosbu

</p>
</details>

<details><summary>Setting waypoints</summary>
<p>
  
- **BUG: Expected value: Line 1 Col 1** - Make sure that the .json file has minimum {} in it

Steps:

1) cw to enter root workspace
  
2) Walk to waypoint

3) Press p. Select table number

</p>
</details>

<details><summary>Navigation</summary>
<p>

#### Getting data from /odom topic
  
```console
   geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.6580884139688824, y=-0.10369131549389796, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=-0.7199514315963468, w=0.6940244492396294))
```   
  
   Turning anti-clockwise: Orientation (Z) increase, Orientation (W) decrease
   
   Turning clockwise: Orientation (Z) decrease, Orientation (W) increase

</p>
</details>

<details><summary>Map</summary>
<p>

#### Saving map

Open rviz when robot is in either (a)gazebo or (b)physical environment and let the robot roam to explore the map

To save the map into the path defined

    ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/install/map_loader/share/map_loader/launch/<map>


#### Loading saved map

Configuration file for map can be found in src/map_loader/launch/load_map.launch.py
   
terminal 1:

    grslam

terminal 2:

    ros2 launch src/map_loader/launch/load_map.launch.py


</p>
</details>


