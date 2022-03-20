## Control without Unity simulation robot
# launch moveit stuff and unity communication
roslaunch vr_hand demo.launch

# visualize hand key points
rosrun vr_hand finger_tip_visualize.py

# HandIK
rosrun vr_hand dynamic_tracking.py

## Control the simulation robot in Unity
