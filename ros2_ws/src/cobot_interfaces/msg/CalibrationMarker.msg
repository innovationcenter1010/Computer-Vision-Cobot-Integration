# One calibration_sample's marker measurement, published by localization_node
# on /vision/calibration_marker (latched, one per sample) so the dashboard can
# show WHERE the marker was seen and whether the sample was accepted — the
# operator's verification view for the automatic (detector) calibration.
std_msgs/Header header

bool reset                 # true once at calibration_begin: forget previous markers
string mode                # "manual" | "automatic"
string model_id            # detector model (automatic) or ""

uint32 sample_index        # 1-based accepted-sample number (0 when rejected)
bool accepted
string message             # why it was rejected, or "px (u, v) d=... mm"

float32 u                  # marker centre, COLOR image pixels
float32 v
float32 depth_mm           # sampled depth at (u, v); -1 when none
float32 confidence         # best detection confidence (automatic); 0 manual
uint32 hits                # detector reads that saw the marker
uint32 reads               # detector reads attempted

float64 tcp_x              # commanded TCP for this pose, robot base mm
float64 tcp_y
float64 tcp_z

uint32 image_width         # of the detections below (0 = unknown)
uint32 image_height
ObbDetection[] detections  # raw boxes of the last detector read (automatic)
