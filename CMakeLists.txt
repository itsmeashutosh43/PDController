cmake_minimum_required(VERSION 2.8.3)
project(pd_controller)
set(pose_follower_ROS_DEPS nav_core base_local_planner costmap_2d roscpp tf nav_msgs pluginlib dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${pose_follower_ROS_DEPS})



generate_dynamic_reconfigure_options(
  cfg/pd_controller.cfg
)


catkin_package(
  LIBRARIES pd_controller
  CATKIN_DEPENDS ${pose_follower_ROS_DEPS}
)

# make sure configure headers are built before any node using them




include_directories(include ${catkin_INCLUDE_DIRS})
add_library(pd_controller src/pd_controller.cpp src/pid_implementation.cpp src/smoother.h)
add_dependencies(pd_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pd_controller ${catkin_LIBRARIES})








install(TARGETS pd_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES pd_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)