# launch moveit group and rviz and hand tracking visualization
roslaunch vr_hand visualize_finger_tip.launch

# BioIK service
rosrun bio_ik_service bio_ik_service

# hand IK solution
rosrun vr_hand hand_ik.py
