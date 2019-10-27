# ROS Beginner Tutorials

Simple publisher and subscriber nodes created by following the [ROS tutorials](https://wiki.ros.org/ROS/Tutorials/).

## Dependencies

### Install ROS

Install ROS Kinetic on Ubuntu system (full installation guide at [ROS installation page](https://wiki.ros.org/kinetic/installation/Ubuntu)):

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```

### Create Catkin Workspace

Create a Catkin workspace if one does not already exist:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

source devel/setup.bash
```

## Installation

Clone repo to Catkin Workspace:

```
cd ~/catkin_ws/src/
git clone https://github.com/cunninghamr/beginner_tutorials.git
```

## Build

Build the publisher and subscriber nodes:

```
cd ~/catkin_ws/
catkin_make
```

## Run

Setup catkin workspace:

```
cd ~/catkin_ws/
source ./devel/setup.bash
```

Start roscore:

```
roscore
```

In a new terminal, run the publisher:

```
rosrun beginner_tutorials talker
```

The publisher should start emitting messages similar to:

```
[ INFO] [1572143969.705265047]: Ryan Cunningham's custom message 155
[ INFO] [1572143969.805112668]: Ryan Cunningham's custom message 156
[ INFO] [1572143969.905326892]: Ryan Cunningham's custom message 157
```

In a new terminal, run the subscriber:

```
rosrun beginner_tutorials listener
```

The subscriber should start emitting messages similar to:

```
[ INFO] [1572143964.605856067]: I heard: [Ryan Cunningham's custom message 155]
[ INFO] [1572143964.705806468]: I heard: [Ryan Cunningham's custom message 156]
[ INFO] [1572143964.805783419]: I heard: [Ryan Cunningham's custom message 157]
```
