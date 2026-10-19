# The typical nav_msgs/msg/Odometry message, but with covariance removed from pose and twist, and no child_frame_id
# Pose will always be treated as if it's in a global frame
# Twist will always be treated as if it's in a global frame, and if given, it will override path planning and instead use the provided linear and angular velocities

# Includes time information of when message is sent

std_msgs/Header header

# Global pose to go to

geometry_msgs/Pose pose

# Global twist to override movement. If all linear and angular velocity values are 0, then it will instead apply path planning to go to the provided pose

geometry_msgs/Twist twist
