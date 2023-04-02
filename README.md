# turtlebot_ws

HOW TO GUIDE

<details><summary>Tutorial</summary>
<p>

- Make sure laptop and pi both connected to same network(hotspot)

- Open 1st Terminal: SSHRP -> ROSBU after the above is done

- Open 2nd Terminal: ```rslam```

- Open 3rd Terminal: 
  map2base

- Open 4th Terminal: 
  setwp



</p>
</details>


<details><summary>Controls</summary>
<p>

- **w/a/x/d/s** - forward/left/backward/right/stop (left and right turns indefinitely, use +/- integers to turn by specific values)

- **p** - setting current coordinate as waypoint. Input table number to know which table this waypoint will lead to. See more at **Setting waypoints**

- **negative integer** - turn right/CW by (-1 to -180deg)

- **postive integer** - turn left/ACW by (1 to 180deg)



</p>
</details>

<details><summary>Setting waypoints</summary>
<p>
  
- **BUG: Expected value: Line 1 Col 1** - Make sure that the .json file has minimum {} in it

Steps:
  
alias: setwp

1) cw to enter root workspace. ros2 run auto_nav setWaypoints
  
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

   Original O(Z): 0  and O(W): 1
  
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


<details><summary>Troubleshooting</summary>
<p>
  
- **No map received** - restart rslam and rosbu, make sure that you rosbu first then rslam
- **Connection Refused** - check same hotspot(NOT the nus), restart hotspot
- **RSLAM not working well** - Re-rosbu

</p>
</details>
