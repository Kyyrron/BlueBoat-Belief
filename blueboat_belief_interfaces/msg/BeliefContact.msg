# One Bernoulli contact of BlueBoat-Belief (contract belief_grid/0). Map frame, metres.
uint8 STATE_CANDIDATE=0
uint8 STATE_CONFIRMED=1
uint8 SECTOR_UNKNOWN=255
uint32 uid                                # stable, never reused (also after pruning)
float64 x
float64 y
float64[4] covariance                     # row-major [xx, xy, yx, yy] m^2, symmetric PSD
float32 p_exists                          # sigmoid of the contact log-odds (not a probability while the detector is uncalibrated)
uint8 state
float32[] class_probabilities             # aligned with BeliefGrid.class_names
uint8[] sectors_detected                  # unique, sorted
uint8[] sectors_missed                    # unique, sorted (sector-centre convention)
uint16 n_looks_detected
float32 first_confidence                  # score of the creating detection; NaN unknown
uint8 first_sector                        # sector of the creating detection; 255 unknown
float32 max_confidence
bool shadow_known                         # false: has_shadow meaningless
bool has_shadow
builtin_interfaces/Time created
builtin_interfaces/Time updated
float32[] look_bearings_deg               # per look: directed look bearing (deg CCW from East)
float32[] look_ranges_m                   # per look: ground range boat -> contact
bool[] look_detected                      # per look: detected (true) or missed (false); added to belief_grid/0
string[] detection_ids                    # "<array stamp ns>:<Detection2D.id>" (ids repeat across GCS streams)
