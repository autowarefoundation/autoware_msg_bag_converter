cmake_minimum_required(VERSION 3.14)
project(autoware_internal_planning_msgs_v1_13)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CandidateTrajectory.msg"
  "msg/CandidateTrajectories.msg"
  "msg/GeneratorInfo.msg"
  "msg/ScoredCandidateTrajectory.msg"
  "msg/ScoredCandidateTrajectories.msg"

  DEPENDENCIES
    std_msgs
    unique_identifier_msgs
    autoware_planning_msgs

  ADD_LINTER_TESTS
)

ament_auto_package()
