cmake_minimum_required(VERSION 2.8.3)
project(rtt_telemanip_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  telemanip_msgs
  rtt_std_msgs  rtt_geometry_msgs  rtt_cartesian_trajectory_msgs)

ros_generate_rtt_typekit(telemanip_msgs)
ros_generate_rtt_service_proxies(telemanip_msgs)

catkin_package(
  DEPENDS ocl rtt
  CATKIN_DEPENDS 
  rtt_roscomm 
  telemanip_msgs
  rtt_std_msgs  rtt_geometry_msgs  rtt_cartesian_trajectory_msgs)

orocos_generate_package()
