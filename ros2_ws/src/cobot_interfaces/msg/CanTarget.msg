# One pick target, ROBOT SPACE (mm / degrees, robot base frame).
# Field names intentionally match the legacy OBBDetection attributes so
# cobot code consuming them (real_x, real_y, real_rz) ports unchanged.

int32 class_id
float32 confidence

float64 real_x
float64 real_y
float64 real_z      # robot base-frame Z of the can top, from the fitted
                    # camera->robot calibration + depth at the can center (mm)
float64 real_rz     # rotation about z, degrees

int32 detection_index -1   # index into the ObbDetectionArray this target came
                           # from (set by localization_node); -1 = unknown
