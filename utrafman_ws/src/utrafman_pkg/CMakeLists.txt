cmake_minimum_required(VERSION 3.8)
project(utrafman_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake   REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(Eigen3        REQUIRED)
find_package(gazebo_ros    REQUIRED)
find_package(gazebo_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(utrafman_msgs REQUIRED)




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



# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



# Add header files for ROS2 UTRAFMAN messages and services
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/utrafman_msgs/rosidl_generator_cpp/)
#message("=====================UTRAFMAN ==== CMAKE_CURRENT_SOURCE_DIR  =  ${CMAKE_CURRENT_SOURCE_DIR}")
# ¡¡Este mensaje no sale por pantalla. Sale en la consola de Visual Studio Code!!




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




add_library(HelloWorld SHARED src/HelloWorld.cc)
ament_target_dependencies(HelloWorld ${ROS_LIBS})
target_link_libraries(HelloWorld ${GAZEBO_LIBRARIES})



add_library(AirSpaceManager SHARED src/AirSpaceManager.cc)
ament_target_dependencies(AirSpaceManager ${ROS_LIBS} utrafman_msgs)
target_link_libraries(AirSpaceManager ${GAZEBO_LIBRARIES})



# add_library(DroneControl SHARED src/dronecontrol.cc)
# ament_target_dependencies(DroneControl ${ROS_LIBS})
# target_link_libraries(DroneControl ${GAZEBO_LIBRARIES})


# Install targets
install(TARGETS
  HelloWorld
  AirSpaceManager
# DroneControl
  DESTINATION lib/${PROJECT_NAME}
)



# Install directories
install(DIRECTORY 
  # launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)



# Declare ROS 2 package
ament_package()
