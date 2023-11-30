cmake_minimum_required(VERSION 3.8)
project(utrafman_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake   REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_ros    REQUIRED)
find_package(gazebo_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3        REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Operator.msg"
  "msg/UAV.msg"
  "msg/Telemetry.msg"
  "msg/Waypoint.msg"
  "msg/Uplan.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



# Declare ROS 2 package
ament_package()


# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})


# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Add ROS 2 libraries
set(ROS_LIBS
  rclcpp
  std_msgs
  gazebo_ros
#  geometry_msgs
)

# Add executable targets

# add_library(DroneControl SHARED src/dronecontrol.cc)
# ament_target_dependencies(DroneControl ${ROS_LIBS})
# target_link_libraries(DroneControl ${GAZEBO_LIBRARIES})

add_library(UTRAFMAN_gazebo SHARED src/utrafman_gazebo.cc)
ament_target_dependencies(UTRAFMAN_gazebo ${ROS_LIBS})
target_link_libraries(UTRAFMAN_gazebo ${GAZEBO_LIBRARIES})

add_library(HELLO_WORLD SHARED src/hello_world.cc)
ament_target_dependencies(HELLO_WORLD ${ROS_LIBS})
target_link_libraries(HELLO_WORLD ${GAZEBO_LIBRARIES})


# Install targets
install(
  TARGETS
#    DroneControl
    UTRAFMAN_gazebo
    HELLO_WORLD
  DESTINATION lib/${PROJECT_NAME}
)





# Install directories
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

