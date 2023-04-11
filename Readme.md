# [Decentralised and Cooperative Control of Multi-Robot Systems through Distributed Optimisation](https://arxiv.org/abs/2302.01728)

Yi Dong, Zhongguo Li, Xingyu Zhao, Zhengtao Ding, Xiaowei Huang

## Before you start
Makesure you are running on Ubuntu 18.04 with ROS melodic and Gazebo 9

## Quick Start
<!-- 
```
make build
xhost +
make start
cd /home && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd /home && catkin_make
```

for new windows:
docker exec -it 1e7e95ab41da bash -->

```
cd ROS_ws/src && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd .. && catkin_make

source devel/setup.bash

roslaunch consensus 4turtlebot.launch
```
### Open a new windows
```
source devel/setup.bash
rosrun consensus control_robots.py
```



<!-- ## Docker Version Updates -- 
Still under test
Coming soon -->
<!-- ## Before you start
Please finish docker installation, official website: https://docs.docker.com/engine/install/ -->

<!-- 
```
make build
xhost +
make start
cd /home && git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd /home && catkin_make
```

for new windows:
docker exec -it 1e7e95ab41da bash -->