cmake_minimum_required(VERSION 2.8.3)
project(plan_grasp)

catkin_package()


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(PCL 1.8 REQUIRED)
include_directories(
# include
 ${PCL_INCLUDE_DIRS} 
 ${Boost_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(plan_grasp src/grasp_planner.cpp)
target_link_libraries(plan_grasp ${Boost_LIBRARIES})
target_link_libraries(plan_grasp ${catkin_LIBRARIES})
