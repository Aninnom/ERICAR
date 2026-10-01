cmake_minimum_required(VERSION 3.8)
project(autorace_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 토픽 명세가 확정되면 msg/*.msg를 추가하고 아래 주석을 푼다
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/LaneInfo.msg"
#   DEPENDENCIES std_msgs
# )

ament_package()
