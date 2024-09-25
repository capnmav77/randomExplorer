# Single Agent Exploratoin in a 2D Grid World

## turtlebot3 startup Node : 
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch
```


## turtlebot3 keyboard teleop Node : 
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```


## turtlebot3 slam navigation Node : 
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```


## turtlebot3 navigation Node : 
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```


## Explorer Node Node : 
```bash
ros2 run my_explorer explorer
```


## final Map saver: 
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```
