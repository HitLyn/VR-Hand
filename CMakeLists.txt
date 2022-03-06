cmake_minimum_required(VERSION 3.1.3)
project(vr_hand)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES
    HandTipPose.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
    message_runtime)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
