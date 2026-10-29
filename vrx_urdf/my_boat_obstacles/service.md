Service: /obstacle_spawner/run
Type: std_srvs/srv/Trigger
What it does: Starts all obstacles moving together, 0.5 s after the call (run_lead_s). Calling it
again restarts from the start positions. Refused until every obstacle has spawned.
────────────────────────────────────────
Service: /obstacle_spawner/reset
Type: std_srvs/srv/Trigger
What it does: Puts all obstacles back at their start positions, where they hold still.
────────────────────────────────────────
Service: /obstacle_spawner/set_start
Type: ros_gz_interfaces/srv/SetEntityPose
What it does: Gives one obstacle a new start position and heading without respawning it. Not saved to
the config file.
────────────────────────────────────────
Service: /obstacle_spawner/reload
Type: std_srvs/srv/Trigger
What it does: Re-reads the config file and applies only what changed. A file with any mistake changes
nothing.

Commands

# start moving
ros2 service call /obstacle_spawner/run std_srvs/srv/Trigger

# back to start, holding
ros2 service call /obstacle_spawner/reset std_srvs/srv/Trigger

# move one obstacle's start (orientation = heading it sets off in; omit for east)
ros2 service call /obstacle_spawner/set_start ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: centre_crosser}, pose: {position: {x: -470.0, y: 150.0}, orientation: {z: 0.7071, w: 0.7071}}}"

# apply edits to config/obstacles.yaml
ros2 service call /obstacle_spawner/reload std_srvs/srv/Trigger

Quaternions for set_start headi