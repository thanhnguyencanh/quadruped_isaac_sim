# A single detected/known victim.
std_msgs/Header header          # stamp = detection time; frame_id = map (canonical)
int32 id                        # stable victim id; -1 if not yet associated
geometry_msgs/Pose pose         # victim pose in header.frame_id (map)
float32 confidence              # detector confidence in [0, 1]
string source                   # detector source, e.g. "rgbd", "thermal"
