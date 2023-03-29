cmake_minimum_required(VERSION 2.8.3)
project(inha_aeye_lidar)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    visualization_msgs
  )

find_package (OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include
LIBRARIES opencv
CATKIN_DEPENDS roscpp
               pcl_conversions
               pcl_ros
               visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(run_lidar src/main.cpp)
target_link_libraries(run_lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
