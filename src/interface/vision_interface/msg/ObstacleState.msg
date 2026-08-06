std_msgs/Header header

# False means the perception result must not be used for motion.
bool valid
bool blocked

# Robot-relative angular clearance scan. Angles are radians, positive left.
float32[] directions
float32[] clearances
uint32[] point_counts
bool[] observed

float32 nearest_distance
float32 recommended_direction
float32 max_range
