# High-level intent parsed from natural language commands.

string command_id
string action_type           # e.g., "navigate", "manipulate", "perceive", "report"
string target_object_id      # optional; UUID or semantic label
geometry_msgs/PoseStamped target_pose
string parameters            # JSON-encoded parameters for flexibility
float32 confidence


