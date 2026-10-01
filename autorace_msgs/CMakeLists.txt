cmake_minimum_required(VERSION 3.8)
project(autorace_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 토픽 명세 확정 후 msg/*.msg 추가, 아래 주석 해제
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/LaneInfo.msg"
#   DEPENDENCIES std_msgs
# )

ament_package()
