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
<img src="https://user-images.githubusercontent.com/28807825/144190076-75f45622-e123-4ca4-80cc-7400ba095688.gif" alt="drawing" width="300"/>

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
<p float="left">
  <img src="https://user-images.githubusercontent.com/28807825/144190545-b0bda739-cafd-4825-b2d6-f7887ce020c8.gif" width="300" />
  <img src="https://user-images.githubusercontent.com/28807825/144190558-e8098046-be98-4257-b2c1-c708dba60cae.gif" width="300" /> 
</p>

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
<p float="left">
  <img src="https://user-images.githubusercontent.com/28807825/144191081-11c4f312-0bdb-43c6-bcba-60237e527e4d.gif" width="450" />
  <img src="https://user-images.githubusercontent.com/28807825/144191798-6e759749-4e2e-4a95-bfc1-72a66c26a084.gif" width="450" /> 
</p>


