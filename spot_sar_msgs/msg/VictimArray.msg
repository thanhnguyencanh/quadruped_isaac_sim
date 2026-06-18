# Canonical payload of the /victims topic. A detection frame routinely contains
# multiple victims, so /victims is ALWAYS a VictimArray, never a bare Victim.
std_msgs/Header header
Victim[] victims
