Header header

# End-effector position and velocity
cartesian_trajectory_msgs/PoseTwist posetwist

# TODO: Remove this flag
bool resync_pose

# If true: augment the command with autonomous control
bool augmenter_engaged

# Gripper opening 0.0:closed - 1.0:open
float64 grasp_opening

# Safety flags
bool deadman_engaged # True if the deadman is being held
bool estop           # True if the estop has been pressed
