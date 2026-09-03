# One pick target, ROBOT SPACE (mm / degrees, robot base frame).
# Application-agnostic: a "target" is anything a detector found that the
# cobot can act on (cans today, any other object class tomorrow).
# Field names match the legacy OBBDetection attributes so cobot code
# consuming them (real_x, real_y, real_rz) ports unchanged.

int32 class_id
float32 confidence

float64 real_x
float64 real_y
float64 real_z      # robot base-frame Z of the object top, from the fitted
                    # camera->robot calibration + depth at the object center (mm)
float64 real_rz     # rotation about z, degrees (0 for axis-aligned detectors)

int32 detection_index -1   # index into the ObbDetectionArray this target came
                           # from (set by localization_node); -1 = unknown

# Where this target was seen at scan time, so /vision/verify_target can
# re-sample the exact pixel later instead of inverting the calibration from
# real_x/y/z. Set by localization_node; -1 = unknown (hand-built target).
float32 pixel_x -1.0
float32 pixel_y -1.0
float64 depth_mm -1.0      # raw depth reading at (pixel_x, pixel_y) at scan time, mm
