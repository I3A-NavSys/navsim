cmake_minimum_required(VERSION 3.8)
project(utrafman_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_ros    REQUIRED)
find_package(gazebo_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Operator.msg"
  "msg/UAV.msg"
  "msg/Telemetry.msg"
  "msg/Waypoint.msg"
  "msg/Uplan.msg"

  "srv/DeployUAV.srv"
  "srv/TranslateUAV.srv"
  "srv/RemoveUAV.srv"

  "srv/RegisterOperator.srv"
  "srv/RegisterUAV.srv"
  "srv/RegisterFP.srv"
  "srv/GetOperators.srv"
  "srv/GetUAVs.srv"
  "srv/GetFPs.srv"
  
  "srv/GetTelemetry.srv"
  
  DEPENDENCIES geometry_msgs builtin_interfaces
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
