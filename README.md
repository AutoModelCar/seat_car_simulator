# seat_car_simulator

Developed at **IRI**, Institut de Robòtica i Informàtica Industrial, CSIC-UPC:  
www.iri.upc.edu

Provides a Gazebo simulation of a scaled 1:10 vehicle:  
https://github.com/AutoModelCar/AutoModelCarWiki/wiki

Used in the SEAT Autonomous Driving Challenge:  
http://www.autonomousdrivingchallenge.com

## Installing

Install modelcar basic packages, because we need its odometry package

```
wget http://ftp.imp.fu-berlin.de/pub/autonomos/data/modelcar/software-packages/v3.1/kinetic/amd64/modelcar-basic-packages.deb
sudo dpkg -i modelcar-basic-packages.deb
source /opt/ros/modelcar/catkin_ws/install/setup.bash

```


Create our catkin_ws workspace, overlaying /opt/ros/modelcar/install/catkin_ws

```
source /opt/ros/modelcar/catkin_ws/install/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
# You should add this last line to your ~/.bashrc file
```



Clone simulator in catkin_ws

```
roscd; cd ../src
git clone https://github.com/AutoModelCar/seat_car_simulator.git
rosdep install -i --from-paths seat_car_simulator
roscd; cd ..
catkin_make
export GAZEBO_MODEL_PATH=`rospack find seat_car_gazebo`/models:$GAZEBO_MODEL_PATH
# You should add this last line to your ~/.bashrc file
```

## Running 

```
roslaunch seat_car_gazebo sim.launch

# Test semaphore
rosservice call /semaphore_start1_color_plugin/trigger

# Test car movement
rostopic pub /steering std_msgs/UInt8 "data: 75"
rostopic pub /manual_control/speed    std_msgs/Int16 "data: 200"
```