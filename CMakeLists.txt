cmake_minimum_required(VERSION 3.0.2)
project(a_star_planner)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_ros
  sensor_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    a_star_planner
  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    visualization_msgs
)

# services
add_service_files(
    DIRECTORY srv
    FILES
    MakeNavPlan.srv
)

add_library(a_star_planner src/a_star_planner.cpp)
add_dependencies(a_star_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(a_star_planner ${catkin_LIBRARIES})

install(TARGETS a_star_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES a_star_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
