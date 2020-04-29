#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
roslaunch dt_street_light_controller_camera dt_street_light_controller_camera.launch veh:=$VEHICLE_NAME
