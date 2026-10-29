#!/bin/bash
# spawn_obstacle.sh <pillar|barge|buoy> <name> <x> <y> [yaw_deg]
#
# Spawns one static obstacle into the running sim, in GAZEBO WORLD coordinates
# (the boat spawns near -528, 186). Once placed you can drag it anywhere with
# the GUI Translate tool -- static models keep whatever pose you drop them at.
set -e
TYPE="${1:?usage: spawn_obstacle.sh <pillar|barge|buoy> <name> <x> <y> [yaw_deg]}"
NAME="${2:?need a unique model name}"
X="${3:?need x}"; Y="${4:?need y}"; YAW_DEG="${5:-0}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILE="$DIR/obstacle_${TYPE}.sdf"
[ -f "$FILE" ] || { echo "no such obstacle type: $TYPE"; exit 1; }
YAW=$(python3 -c "import math;print(math.radians($YAW_DEG))")
export GZ_VERSION=garden
ros2 run ros_gz_sim create -world sydney_regatta -file "$FILE" \
     -name "$NAME" -x "$X" -y "$Y" -z 0 -Y "$YAW"
