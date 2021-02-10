# time
float64 car2_t0
float64 car3_t0
float64 traj_t0

# position
float32 x0
float32 y0
float32 yaw0
float32 v0
float32 a0

# planned trajectories
object_msgs/Trajectory[] planned_trajectories
object_msgs/PolygonArray[] ego_polygons
float32[] costs
int32 best_ind
float32 best_cost

# predictions
object_msgs/DebugPrediction[] car2_predictions
object_msgs/DebugPrediction[] car3_predictions
object_msgs/Polygon[] car2_polygons
object_msgs/Polygon[] car3_polygons