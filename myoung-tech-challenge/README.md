# Technical Challenge

Completed coding challenge for a recent technical interview. The challenge summary was:

> Launch a turtlesim simulation using ROS Noetic (and Ubuntu 20.04).
> Write a node in C++ that can receive input commands from two sources:
>
> -​ Waypoints from a topic and generate a path to it.
> -​ Velocity commands from the keyboard.
>
> The node should be able to handle both requests and prioritise the commands from the
> keyboard. This means that if a keyboard message comes in and the other action is being
> executed, stop it.


# Installation & Startup

1. Follow the instructions for installing ROS1 Noetic
1. Clone the workspace
   ```
   git clone git@github.com:mattforestyoung/misc_projects.git
   ```
1. Source your ROS workspace
   ```
   source /opt/ros/noetic/setup.bash
   ```
1. Build and source the turtlesim_path_planner package
   ```
   cd ~/myoung-tech-challenge
   catkin_make
   source devel/setup.bash
   ``` 
1. To start the turtle simulator and path planner, run each of the following commands in three separate terminals
   ```
   roscore
   rosrun turtlesim turtlesim_node                           
   rosrun turtlesim_path_planner turtlesim_path_planner_node
   ```
1. Follow the usage instructions below to learn how you can manually control the turtle, or set a series of waypoints for it to autonomously navigate to

# Usage
## Manual control 
If you select the terminal in which you launched the planner node, you can use these keys to manually move the turtle around:
```
Up    : move forward
Down  : move backward
Left  : rotate counter-clockwise
Right : rotate clockwise

q/z   : increase/decrease max speeds by 10%
w/x   : increase/decrease only linear speed by 10%
e/c   : increase/decrease only angular speed by 10%
```
(I.e. if you select another window or terminal, the planner node will not receive the key presses)

The planner node will print out a copy of the full instructions on startup.

## Waypoint control

To send waypoints, you will need to open a new terminal and use the following example command to publish a waypoint to the `/turtle1/waypoint` topic. The planner node will listen for it, and automatically plot a path to that coordinate:
```
rostopic pub /turtle1/waypoint turtlesim/Pose "{x: 4.0, y: 10.0}"
```
The planner node can listen for and queue several waypoints, and will navigate to each in the order in which they are received. If the planner node receives a keyboard move command, this will take priority, and the current waypoint will be paused. Approximately 3 seconds after the last valid move key is pressed, the planner will resume navigation to the previously paused waypoint. The planner node will print information to terminal indicating when waypoints are started, paused or resumed.

When picking coordinates for waypoints, the notes below on how the turtlesim environment may be of use. 

## Notes on the turtlesim environment
* The default coordinate frame the turtle spawns into has a height and width of approximately 12 units 
* The turtle spawns approximately at the coordinates [5.5, 5.5]
* The axes are East-North-Up, as per [ROS REP-103](https://www.ros.org/reps/rep-0103.html)
* The yaw of the turtle is zero facing right (east), and runs from 0.0 to +/- Pi (3.14), increasing counter-clockwise, and decreasing clockwise

## Other useful commands
* Reset the location of the turtle:
   ```
   rosservice call /reset
   ```
* Teleport the turtle to an absolute location:
   ```
   rosservice call /turtle1/teleport_absolute x y theta
   ```

## Using RQT
You can control and view the turtle data using `rqt`. Launch it in a new terminal with the command:
```
rqt
```
And then go to Perspectives > Import and select the file `turtlesim_rqt_view.perspective` from the same folder as this readme. This will load several plugins into rqt, which can allow you to directly view the turtlesim node graph, topics, and even control the turtle manually with the sliders in the "Robot Steering" plugin. 

## TODO
- [X] Outline basic structure and class
- [X] Set up listener to /turtle1/pose
- [X] Set up listener for waypoints
- [X] Set up publisher to /turtle1/cmd_vel
- [X] Create proportional controller based on reference code 
- [X] Create checking function for whether a waypoint has been reached
- [X] Implement sequential waypoint following
- [X] Create listener for keyboard input, use (roughly) the same commands as [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)
- [X] Implement queue for waypoints
- [X] Add timer to resume/recalculate most recent waypoint 
- [X] Add Doxygen commenting
- [X] Add instructions for viewing everything in rqt_plot
- [X] Tidy up instructions
- [X] Tidy up code