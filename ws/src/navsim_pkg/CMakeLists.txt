cmake_minimum_required(VERSION 3.8)
project(navsim_pkg)

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
find_package(navsim_msgs   REQUIRED)




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




# Add header files for ROS2 NAVSIM messages and services
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/navsim_msgs/rosidl_generator_cpp/)
#message("=====================NAVSIM ==== CMAKE_CURRENT_SOURCE_DIR  =  ${CMAKE_CURRENT_SOURCE_DIR}")
# ¡¡Este mensaje no sale por pantalla. Sale en la consola de Visual Studio Code!!




# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})


# Set compiler flags
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -g")



# Add ROS 2 libraries
set(ROS_LIBS
  rclcpp
  std_msgs
  gazebo_ros
  geometry_msgs
)



# Add executable targets

add_library(World SHARED plugins/World.cc)
ament_target_dependencies(World ${ROS_LIBS} navsim_msgs)
target_link_libraries(World ${GAZEBO_LIBRARIES})

add_library(DCdrone SHARED plugins/DCdrone.cc)
ament_target_dependencies(DCdrone ${ROS_LIBS})
target_link_libraries(DCdrone ${GAZEBO_LIBRARIES})



# Install targets
install(TARGETS
  World
  DCdrone
  # dronecontrol_copy
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
