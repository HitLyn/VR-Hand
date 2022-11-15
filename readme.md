# VR-ShadowHand
Control UR10-Shadow arm-hand system with Oculus

## Requirements
Linux machine (Ubuntu, ROS)
Windows machine (ROS, Unity, Oculus)
Oculus Quest2 + Oculus Link Cable

## Hardware Connection
Link Oculus Quest2 with Windows using the Oculus Link Cable, the linux machine and the Windows machine should be in the same local network.

## Installation
On Ubuntu:
1. Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Download [VR-Hand ROS package](https://drive.google.com/file/d/1oe7UmbuDleIHNe2Oh04PIiQn4MOv_BCF/view?usp=share_link), create ROS workspace and compile.

On Windows:
1. Install Unity 2020.3.26f1
2. Install [ROS](http://wiki.ros.org/Installation/Windows)
3. Set the same ROS master URL for Windows and Ubuntu
4. Download the Unity package [VR-Shadow-Teleoperation](https://drive.google.com/file/d/1VE7xPYl_VV5D8OJ-HcyLGxFncXLTzBd2/view?usp=share_link)


## Usage
On Ubuntu:

```shell
roslaunch vr_hand demo.launch
rosrun vr_hand finger_tip_visualize.py
rosrun vr_hand hand_control.py
```

On Windows:

1. Link the Oculus Quest2 to Windows, choose the `Oculus Link` mode in the Oculus.
2. Start Unity, load the `VR-Shadow-Teleoperation` package
3. Install [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
4. Follow the [instructions](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/2_ros_tcp.md) and generate ROS message `JointPositionsMsg` (located [here](https://github.com/HitLyn/VR-Hand/tree/master/msg)) in Unity  
5. Open the `SampleScene` and click `play`
