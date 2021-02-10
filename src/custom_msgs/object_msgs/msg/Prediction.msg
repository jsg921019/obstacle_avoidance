std_msgs/Header header
uint32 id

uint32 n_predictions
float32 dt  # s
object_msgs/Object[] predictions

float32[] sigx
float32[] sigy
float32[] rho
time t0  # time corresponding prediction[0]