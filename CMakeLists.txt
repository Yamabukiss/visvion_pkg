cmake_minimum_required(VERSION 3.0.2)
project(vision_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp dynamic_reconfigure std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/dynamic.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_pkg
#  CATKIN_DEPENDS cv_bridge roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})