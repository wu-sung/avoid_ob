cmake_minimum_required(VERSION 3.5)
project(avoid_ob)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # Dynamixel SDK 패키지 찾기
# Include directories
include_directories(include)
include_directories(/opt/ros/foxy/include/dynamixel_sdk)
# Build

include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
  include
)
add_executable(sub src/sub.cpp src/main.cpp ~/ros2_ws/src/avoid_ob/src/lidarsave.mp4)
ament_target_dependencies(sub
  rclcpp
  std_srvs
  sensor_msgs
  OpenCV
)

install(
  TARGETS   
  sub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(OpenCV)
ament_export_include_directories(include)
ament_package()
