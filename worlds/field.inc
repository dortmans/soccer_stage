define floorplan model
(
  # sombre, sensible, artistic
  color "gray30" # see http://www.astrouw.edu.pl/~jskowron/colors-x11/rgb.html

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_outline 1
  gui_move 0
  gripper_return 0
  fiducial_return 0
  ranger_return 1
  obstacle_return 1
)

define zone model
(
  gui_nose 0
  gui_grid 1
  gui_outline 0
  gui_move 0
  gripper_return 0
  fiducial_return 0
  ranger_return -1.000  # transparent to range sensors
  obstacle_return 0 # insensible to collision
)

define goal_post model
(
  size [0.125 0.125 1.0]
  gui_outline 1
  gui_move 0
  ranger_return 1
  obstacle_return 1
  color "white"
)

define crossbar model
(
  size [2.65 0.125 0.125]
  gui_outline 1
  gui_move 0
  ranger_return 1
  obstacle_return 1
  color "white"
)