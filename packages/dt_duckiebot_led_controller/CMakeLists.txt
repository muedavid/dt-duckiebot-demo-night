cmake_minimum_required(VERSION 2.8.3)
project(dt_duckiebot_led_controller)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  rospy
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Light_Adjustment.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  duckietown_msgs
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
