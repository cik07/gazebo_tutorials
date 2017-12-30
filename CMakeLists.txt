cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  pcl_ros
  pcl_conversions
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

#add_library(gazebo_tutorials src/simple_world_plugin.cpp)
#target_link_libraries(gazebo_tutorials ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_library(model_push src/model_push.cpp)
target_link_libraries(model_push ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(joy src/joy.cpp)
target_link_libraries(joy ${catkin_LIBRARIES})

add_executable(pcl src/pcl.cpp)
target_link_libraries(pcl ${catkin_LIBRARIES})

add_executable(ilk src/pcl_ilk.cpp)
target_link_libraries(ilk ${catkin_LIBRARIES})

add_executable(otonom src/move_otonom.cpp)
target_link_libraries(otonom ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
