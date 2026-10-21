cmake_minimum_required(VERSION 3.8)
project(inspection_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) #接口生成器
find_package(builtin_interfaces REQUIRED)  #提供time类型库

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


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InspectionSample.msg"
  "msg/HealthState.msg"
  "msg/TargetStation.msg"
  "msg/InspectionReport.msg"
  "msg/ArrivalEvent.msg"
  "msg/MissionStatus.msg"
  "msg/MissionStatusArray.msg"
  "msg/QueueEntry.msg"
  "msg/QueueStatus.msg"
  "msg/RobotOccupancy.msg"

  "srv/GetInspectionReport.srv"
  "srv/SetInspectionPause.srv"
  "srv/ResetRobotAlarm.srv"
  "srv/GetRobotHealth.srv"

  "srv/GetStation.srv"
  "srv/SetMissionPause.srv"
  "srv/GetMissionState.srv"
  "srv/GetMissionQueue.srv"

  "action/CollectStation.action"
  
  "action/InspectRoute.action"
  DEPENDENCIES builtin_interfaces
) #将接口文件生成代码

ament_export_dependencies(rosidl_default_runtime)

ament_package()
