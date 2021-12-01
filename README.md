# ROS2 Practice Example
This repository is for practicing of some ros2 feature.   
The publisher and subscriber of three different sensors are implemented.   
The publisher is written in C++ and the subscriber is written in Python for eaiser usage of matplotlib visualization.  
Each example contains some demonstrations:  
- [**Temperature Sensor**](#temperature-sensor)  
  * Publish simulated temperature of gaussian distribution   
  * Using custom message type   
  * Visualize received data with matplotlib  
- [**Speed Sensor**](#speed-sensor)  
  * Publish simulated speed with gaussian noise  
  * Using geometry_msgs/Twist message type
  * Visualize received data with turtlesim and matplotlib  
- [**Laser Sensor**](#laser-sensor)  
  * Parse and publish pre-recored rosbag2 data
  * Using sensors_msgs/LaserScan message type  
  * Visualize message data with RVIZ  

## Environment
Ubuntu 20.04 (on Windows11 wsl2)  
ROS2 foxy

## File Structure
```python
├── data_processor # Sensors subscriber and visualization tools  
│   ├── data_processor  
│   │   └── utils  
│   ├── launch  
│   ├── resource  
│   ├── rviz # Rviz config file  
│   └── test 
├── sensor_interfaces # Custom temperature sensor data  
│   ├── include  
│   │   └── sensor_interfaces 
│   ├── msg  
│   └── src  
└── sensors  # Sensors publisher and launch file  
    ├── include  
    │   └── sensors  
    ├── laser_test_data # Test data for laser sensor 
    ├── launch  
    └── src  
```
## Build
Clone project into src of ros2 workspace 
```bash
cd {ros2_workspace}/src
git clone 
```
Go back to ros2 workspace and build
```bash
cd {ros2_workspace}
colcon build
```
## Run
### Temperature Sensor
Run nodes seperately  
```bash
ros2 run sensors temp_publisher  
ros2 run data_processor temp_subscriber 
```
Launch all nodes 
```bash
ros2 launch sensors temp.launch
```

### Speed Sensor
Run nodes seperately  
```bash
ros2 run sensors speed_publisher  
ros2 run data_processor speed_subscriber 
```
Launch all nodes 
```bash
ros2 launch sensors speed.launch
```

### Laser Sensor
Run nodes seperately  
```bash
ros2 run sensors laser_publisher  
ros2 run data_processor laser_subscriber 
```
Launch all nodes 
```bash
ros2 launch sensors laser.launch
```



