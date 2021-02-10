cmake_minimum_required(VERSION 3.0.2)
project(object_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

set(MSG_FILES
  "Object.msg"
  "Prediction.msg"
  "Control.msg"
  "Trajectory.msg"
  "trajectory_array.msg"
  "trajectory.msg"
  "Predictions.msg"
  "DebugPrediction.msg"
  "PlanningDebug.msg"
  "Polygon.msg"
  "PolygonArray.msg"
)


add_message_files(FILES
    ${MSG_FILES}
    DIRECTORY msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)


catkin_package(CATKIN_DEPENDS
 geometry_msgs
 std_msgs
 message_runtime
)
