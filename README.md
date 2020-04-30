# Obstacle-Avoidance-Bot-Using-ROS
This project contains a bot simulation in Gazebo using bug0 algorithm. The robot design and other requirements can be found in this [PDF](assignment.pdf)
In this program the bot reaches the given goal by following the boundary of the obstacle if any. All codes are written in python.
Xacro is used in the project to clean the URDF code.

## Getting Started

1. Clone this repository in the `src` folder of your `catkin` workspace
2. Run `catkin_make`
3. Go to obstacle-avoid.py and make it executable using `chmod +x obstacle-avoid.py`. No need to do this step if you are using the obstacle-avoid.cpp code.
3. Open 3 Terminals
4. Run the command `roslaunch Obstacle-Avoidance-Bot-Using-ROS spawn.launch`. 
5. In the second terminal run the command `rosrun Obstacle-Avoidance-Bot-Using-ROS obstacle-avoid.py `
6. In last terminal, run the command `rosrun gazebo_ros gazebo --verbose` to start the robot and begin the obstacle avoidance course.
7. Add blocks in between wherever you want or you can download a world from anywhere else and spawn the bot there.

## Prerequisites

* [ROS](http://wiki.ros.org/kinetic)  
* [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)


## Video

[![Obstacle-Avoidance-Using-ROS-And-Gazebo](http://img.youtube.com/vi/yoHwEvan2nE/0.jpg)](https://www.youtube.com/watch?v=yoHwEvan2nE "Obstacle-Avoidance-Using-ROS-And-Gazebo")

## To-Do / Improvements

* Make the bot move in the shortest distance

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
