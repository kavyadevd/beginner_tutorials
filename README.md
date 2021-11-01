# ROS Beginner Tutorials

A beginner ROS package that implements simple publisher-subscriber nodes in ROS


## Installations

To install ROS refer [this link](http://wiki.ros.org/ROS/Installation).

Preferred Ubuntu 18.04, ROS Melodic


### Step 1. Clone repository and build the package
```bash
git clone --recursive https://github.com/kavyadevd/beginner_tutorials.git
cp <repository_path> <catkin_workspace_path/src/>
cd <catkin_workspace>
source ./devel/setup.bash
catkin_make
```

### Step 2. Make sure that a roscore is up and running
```bash
roscore
```
### Step 3. In new terminal run publisher node 
```bash
rosrun beginner_tutorials talker
```
### Step 4.  In new terminal run subscriber node 
```bash
rosrun beginner_tutorials listener
```


## Licensing
The project is licensed under [MIT License](https://opensource.org/licenses/MIT). Click [here](https://github.com/kavyadevd/beginner_tutorials/blob/main/LICENSE) to know more
