# One pick target, ROBOT SPACE (mm / degrees, robot base frame).
# Field names intentionally match the legacy OBBDetection attributes so
# cobot code consuming them (real_x, real_y, real_rz) ports unchanged.

int32 class_id
float32 confidence

float64 real_x
float64 real_y
float64 real_z      # measured from depth at the can center (mm)
float64 real_rz     # rotation about z, degrees
