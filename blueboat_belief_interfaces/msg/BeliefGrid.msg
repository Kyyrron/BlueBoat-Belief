# BlueBoat-Belief snapshot, contract belief_grid/0. One self-consistent instant.
# Spec: blueboat_belief/docs/topics.md (layout, conventions, NaN rules) and docs/MATH.md (model).
# Layout: cell i = row*width + col; row 0 = south edge, col 0 = west edge;
# cell centre = (origin_x + (col+0.5)*resolution, origin_y + (row+0.5)*resolution).
# Probability channels are in [0,1] and NaN outside the survey area (and only there).
# Per-sector arrays: index cell*aspect_sector_count + s.
std_msgs/Header header                    # stamp = snapshot time (sonar-profile clock); frame_id "map"
string contract_version                   # "belief_grid/0"
uint32 grid_seq                           # +1 per published snapshot
uint32 width
uint32 height
float32 resolution                        # m per cell
float64 origin_x                          # map-frame lower-left corner of cell (0,0) [m]
float64 origin_y
float64 datum_lat                         # WGS-84 of map (0,0) [deg]
float64 datum_lon
float64 odom_to_map_x                     # x_map = R(yaw)*x_odom + t [m]
float64 odom_to_map_y
float64 odom_to_map_yaw                   # rad, CCW (0: odom yaw is absolute ENU)
float32 odom_to_map_sigma_m               # 1-sigma of the translation; NaN = unknown
bool odom_to_map_valid                    # false until converged: do not plan
uint8 aspect_sector_count                 # S
float32 aspect_sector_origin_deg          # lower edge of sector 0, deg CCW from East, directed look bearing
string[] class_names
float32 prior_density_per_ha
string model_params_json                  # {"schema": 1, ...} informational
float32[] p_background                    # P(undetected target in cell); contacts excluded
float32[] p_target                        # background combined with contacts (rendered)
float32[] coverage                        # 1 - Q: modelled cumulative detection probability
uint16[] n_looks                          # independent looks (synthesis name: n_observations)
uint32[] n_pings                          # raw pings
uint8[] n_aspects                         # distinct sectors with a look
float32[] depth_m                         # water depth of the last observing ping (flat floor); NaN unseen
float32[] last_intensity_db               # mean sample dB of the last observing ping; NaN unseen
float32[] max_confidence                  # best associated detection score; 0 if none
uint8[] aspect_counts                     # width*height*S looks per sector, saturating at 255
float32[] aspect_log_miss                 # width*height*S, ln Q per sector (= ln(1 - max PD)); may be empty
float64[] area_x                          # survey polygon vertices, map frame [m] (added to belief_grid/0)
float64[] area_y
BeliefContact[] contacts
