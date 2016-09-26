Header header

# End-effector position and velocity
cartesian_trajectory_msgs/PoseTwist posetwist

# Gripper opening 0.0:closed - 1.0:open
float64 grasp_opening

# Mode Flags
# If true: feed the commands forward as manual control
bool deadman_engaged
# If true: interpret high-level commands with augmenter classifiers
bool augmenter_engaged

# High-Level Commands
bool emergency_stop      # put the arm into a safe stop mode
bool interrupt_augmenter # preempt any tunning augmenter taskels
bool attempt_rescue      # plan a collision-free path to return the arm to the last commanded pose

# Local stamp (for acking)
time local_stamp
