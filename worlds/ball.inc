define ball position
(
  # cylinder shape
  block
  ( 
    points 16
    point[0]  [ 0.225 0.000 ]
    point[1]  [ 0.208 0.086 ]
    point[2]  [ 0.159 0.159 ]
    point[3]  [ 0.086 0.208 ]
    point[4]  [ 0.000 0.225 ]
    point[5]  [ -0.086 0.208 ]
    point[6]  [ -0.159 0.159 ]
    point[7]  [ -0.208 0.086 ]
    point[8]  [ -0.225 0.000 ]
    point[9]  [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0 1]
  )

  size [ 0.22 0.22 0.22 ] 
  origin [ 0.0 0.0 0.0 0.0 ]
  
  gui_nose 0
  gui_outline 1
  gui_move 1
  ranger_return 1
  obstacle_return 0

  # kinematics
  drive "omni"
  
  # [ xmin xmax ymin ymax zmin zmax amin amax ] x,y,z in meters per second, a in degrees per second     
  velocity_bounds [-20 20 -20 20 0 0 -360 360 ]
  
  # [ xmin xmax ymin ymax zmin zmax amin amax ] x,y,z in meters per second squared, a in degrees per second squared     
  acceleration_bounds [-20 20 -20 20 0 0 -360 360]
  
  localization "gps" # Report error-free position in world coordinates
  #localization "odom" # Position data drifts from the ground truth over time
  
  # odometry error model parameters, only used if localization is set to "odom"
  odom_error [0.01 0.01 0.00 0.01] # proportion of error
  #odom_error [0.01 0.01 999999 999999 999999 0.01]

)
