# launch moveit stuff and unity communication
roslaunch vr_hand demo.launch

# visualize hand key points
rosrun vr_hand finger_tip_visualize.py

# BioIK service
rosrun bio_ik_service bio_ik_service

# HandIK
rosrun vr_hand dynamic_tracking.py
