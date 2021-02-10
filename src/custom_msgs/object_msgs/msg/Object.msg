std_msgs/Header header
uint32 id

# The type of classification given to this object.
uint8 classification
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_CAR=1
uint8 CLASSIFICATION_PEDESTRIAN=2
uint8 CLASSIFICATION_CYCLIST=3

# The detected position and orientation of the object.
float32 x       # m
float32 y       # m
float32 yaw     # rad

float32 v       # m/s
float32 yawrate # rad/s

float32 a      # m/ss
float32 delta  # radian

# size
float32 L     # m
float32 W     # m
