# Laser sensor definition for the Scout Mini
define scout_mini_laser ranger
(
  sensor
  (
    range [ 0.1 20.0 ]  # min and max range in meters
    fov 360.0           # full circle, matches the real RoboSense lidar
    samples 720         # one beam every 0.5 degrees
  )
  color "black"
  size [ 0.05 0.05 0.05 ]
)

# Robot body definition
define scout_mini position
(
  size [ 0.565 0.490 0.270 ]   # actual Scout Mini dimensions in meters
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1                   # draw a nose so you can see which way it faces

  # omni = holonomic drive, all three velocity components are applied
  # diff would silently ignore lateral velocity — wrong for mecanum wheels
  drive "omni"

  localization "odom"
  localization_origin [ 0.0 0.0 0.0 0.0 ]
  odom_error [ 0.05 0.05 0.0 0.02 ]  # realistic noise: 5cm xy, 0.02rad yaw

  scout_mini_laser(
    name "laser"
    pose [ 0.0 0.0 0.15 0.0 ]  # sensor is 15cm above robot center
  )
)
