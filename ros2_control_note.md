### 1-4 Installation and Setup
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```
---
### 2-3 Add a ros2_control Tag in the URDF
- Details see file mobile_base.ros2_control.xacro
---
### 2-4 Add a Controller Config (P1 JointStateBroadcaster)
- Reference: [ros_control](https://github.com/ros-controls/ros2_controllers)
- Create a new package "my_robot_bringup"
'''
ros2 pkg create my_robot_bringup
'''
- Add a "my_robot_controllers.yaml" under "config" folder
- We will ad a node called ContollerManager, which will manage all the controllers and bind with the hardware interface
- Basically what the controllers do is a loop of read, update and write
    - READ something from the feedback or the command
    - UPDATE the next command
    - WRITE the command to the hardware
- In this lesson we will use "joint state broadcaster"
    - It will replace the "fake" joint_state_publisher_gui
    - It will get the joint state from the real hardware and publish it on the joint state topic
---
### 2-5 Add a Controller Config (P2 Diff Drive Controller)
- Details see file my_robot_controllers.yaml
---
### 2-6 Start Everything from the terminal
- We will start nodes one by one
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/yimin_wsl/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"
ros2 run controller_manager ros2_control_node --ros-args --params-file /home/yimin_wsl/ros2_ws/src/my_robot_bringup/config/my_robot_controllers.yaml
            ^^^ pkg name        ^^^executable
# With "ros2 node list" there is another "controller_manager". That is the node name we defined in my_robot_controllers.yaml

# Start the controllers
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller

# Validate whether the controllers are working
ros2 run rviz2 rviz2 -d /home/yimin_wsl/ros2_ws/src/my_robot_description/rviz/urdf_config.rviz 
# Switch the fixed frame to "odom"

# In ros2 topic list there is a topic /diff_drive_controller/cmd_vel
# Its interface is geometry_msgs/msg/TwistStamped
ros2 interface show geometry_msgs/msg/TwistStamped
>   std_msgs/Header header
        builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
        string frame_id
    Twist twist
        Vector3  linear
            float64 x
            float64 y
            float64 z
        Vector3  angular
            float64 x
            float64 y
            float64 z
# Note that there is a header for timestamp

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
                                                                                                                ^^^ This will publish timestamp

```
### 2-7 Launch File for ros2_control
- Details see my_robot_bringup/config/my_robot.launch.xml
- Remember to add dependencies in packages.xml
- Python launch file is also available in the video!
```bash
ros2 launch my_robot_bringup my_robot.launch.xml

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

rqt_graph
```
---
### 2-8 ros2_control Command Line, Debugging and Recap
```bash
# When the controllers are running, this will give out the controllers that are being used
ros2 control list_controllers

# List all possible controllers I can used and has been installed
ros2 control list_controller_types

ros2 control list_hardware_interfaces
>   command interfaces
        base_left_wheel_joint/velocity [available] [claimed] 
        # "claimed" because the diff drive controller is binded with this interface. Normally there should only be one controller controlling ONE command interface
        # But for state interfaces it doesnt matter
        base_right_wheel_joint/velocity [available] [claimed]
        diff_drive_controller/angular/velocity [available] [unclaimed]
        diff_drive_controller/linear/velocity [available] [unclaimed]
    state interfaces
        base_left_wheel_joint/position
        base_left_wheel_joint/velocity
        base_right_wheel_joint/position
        base_right_wheel_joint/velocity

ros2 control list_hardware_components
>   Hardware Component 1
        name: MobileBaseHardwareInterface
        type: system
        plugin name: mock_components/GenericSystem
        state: id=3 label=active
        read/write rate: 50 Hz
        is_async: False
        command interfaces
                base_left_wheel_joint/velocity [available] [claimed]
                base_right_wheel_joint/velocity [available] [claimed]
```