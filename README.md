# Object Follower

This project aims to make the robot follow a particular object while also keeping a minimum safe distance from it.

The software stacks implemented in this project are ROS and Python. We also made use of OpenCV for the object detection part.

We aim to further expand this project and make it a person-follower.


<br>

# Installation

## Pre-Requisites :
- ROS noetic : Refer to the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for installation of ROS noetic.
               
- Catkin workspace : A catkin workspace is a folder where you modify, build, and. install catkin packages. Take a look ak the [official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for instructions regarding creation of a catkin workspace


<br>

## Installation of Virtualenvwrapper, OpenCV, and CV_bridge

Your can refer to [A.T.O.M's wiki](https://atom-robotics-lab.github.io/wiki/setup/virtualenv.html) for installation of the above mentioned packages and libraries.

<br>

## Clone the Object Follower package
Now go ahead and clone this repository inside the "src" folder of the catkin workspace you just created by executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/object_follower.git
```

<br>

## Clone the MR-Robot package
this package provide us the bot which we are gonig to use.
Go inside the "src" folder of the catkin workspace and executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/MR-Robot.git
```

<br>

Now out robot does not have camera in this package so we have to chang branch from main to with_camera.   
Now then go inside MR-Robot package you just created by executing the above command then executing the command given below in your terminal.
```bash
git checkout with_camera 
```

<br>

## Make the package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package.  (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/catkin_ws
catkin_make
```


<br><br>
BAZINGA!! The installation is done and now its time to play around with the robot :)



## Launch

```bash
roslaunch obj_follower obj_follower_sim.launch
```
The above command when executed in the terminal will launch the gazebo simulation and will also start ROS Master.



<img src = "assets/Images/simulation.png" >


## Run the node

```bash
rosrun obj_follower object_follower.py
```

The given command will run the controller script which controls the robot's movements.

<img src = "https://github.com/atom-robotics-lab/object_follower/blob/main/assets/GIF's/robot.gif" >



