[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)
# Beginers Tutorial 
## A simple Week 12 808x:Soft Development

### Dependencies
* Installed on Linux (Ubuntu 22.04)
* Ros 2 Humble installed 
* And a workspace created for ros2. If not then follow the instructions below:
<details>
<summary>How to create a ros Workspace?</summary>

```xml
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

</details>


## Installation of the Turtlebot package
1. All turtlebot3 pakages install
    ```bash
    sudo apt install ros-humble-turtlebot3*
    ```
2. Humble and Gazebo ROS packages
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs 
    ```
3. Set up gazebo model path 
    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
    prefix turtlebot3_gazebo \
    `/share/turtlebot3_gazebo/models/
    ```


### Building this package
These simple steps are to be followed to replicate the work of this repository from scratch

<details>
<summary>Clone this package </summary>
Inside your ros_workspace/src clone the following package

```xml 
git clone https://github.com/amancodeblast/vacuum_cleaner.git
```
make sure you have selected the tag "Week12_HW"
</details>



<details>
<summary>Compile the package </summary>

Complile the package using the command 
```xml
colcon build --packages-select beginner_tutorials
``` 
</details>
<details>
<summary>Source the workspace and run the nodes using  a launch file </summary>

Open a new terminal, navigate to ros2_ws, and source the setup files:

```xml
. install/setup.bash
```
</details>

### Run Instructions 
- Record with ROS Bag

  ```
  . install/setup.bash
  ros2 launch vacuum_cleaner walker.py record:=True
  ```

- To view ros_bag info

  ```
  ros2 bag info vacuum_cleaner_bag
  ```

### To play from ROS Bag

- On a new terminal

  ```
  cd ~/ros2_ws
  . install/setup.bash
  ros2 run vacuum_cleaner walker.py
  ```