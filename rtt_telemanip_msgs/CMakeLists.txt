cmake_minimum_required(VERSION 2.8.3)
project(rtt_telemanip_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm telemanip_msgs)

ros_generate_rtt_typekit(telemanip_msgs)
ros_generate_rtt_service_proxies(telemanip_msgs)

orocos_generate_package(
  DEPENDS telemanip_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
