cmake_minimum_required(VERSION 3.0.2)
project(puru_joystick)

find_package(catkin REQUIRED COMPONENTS

  joy
  roscpp
  
)

catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)
add_executable(puru_joystick src/joy_puru_node.cpp)
target_link_libraries(puru_joystick ${catkin_LIBRARIES})
