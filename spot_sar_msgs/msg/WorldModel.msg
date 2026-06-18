# The small SYMBOLIC world model produced by symbol grounding (Phase 4) and
# consumed by the Task Executive (Phase 6) to generate problem.pddl.
#
# NOTE: first-cut schema — expected to evolve as the PDDL domain is finalised.
# ROS messages have no maps, so relationships are expressed as parallel arrays.
std_msgs/Header header

# --- locations (symbolic places) ---
string[] locations                      # symbolic ids, e.g. "L0".."Ln"
geometry_msgs/Point[] centroids         # metric centroid per location (parallel to locations)
string[] explored                       # subset of `locations` flagged explored

# --- connectivity (undirected edges, listed once each) ---
string[] connected_a                    # parallel to connected_b: edge endpoints
string[] connected_b

# --- robot ---
string robot_location                   # current symbolic location of the robot ("" if unknown)

# --- victims (known so far) ---
Victim[] victims                        # detected victims
string[] victim_location                # parallel to victims: symbolic location of each victim
bool[] victim_reported                  # parallel to victims: reported flag
