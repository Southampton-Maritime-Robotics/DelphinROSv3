# ======= ROS message for changing parameters of Sonar scans ==================
# Used e.g. for bottom tracking, obstacle avoidance

float32 RLim    # Right limit for sonar heading in 1/16th of gradian
float32 LLim    # Left limit for sonar heading in 1/16th of gradian
float32 Heading # NOT CURRENTLY IN USE Sonar heading in 1/16th of gradian
float32 NBins   # Number of bins
float32 Range   # Range in metres

