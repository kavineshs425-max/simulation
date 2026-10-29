# my_boat Sensor Stack (IMU + GPS + LiDAR)

Sensors: **IMU, GPS, and a 3D LiDAR** (LSLiDAR C32). The sensors are ideal to
start with: every available Gazebo noise parameter is present but set to
**zero**, ready to be tuned to the real hardware later.

Localization pipeline these feed (downstream, not part of this phase):
```
Gazebo IMU ─┐
Gazebo GPS ─┴─► navsat_transform_node ─► GPS odom ─► ekf_localization_node ─► /odometry/filtered
```

## 1. Placement (COM-relative, base_link frame: x fwd, y port, z up)

| Link         | Parent      | Joint | x (m) | y (m) | z (m) |
|--------------|-------------|-------|------:|------:|------:|
| `imu_link`   | `base_link` | fixed | 1.30  | 0.08  | −0.07 |
| `gps_link`   | `base_link` | fixed | 0.93  | 0.00  |  0.46 |
| `lidar_link` | `base_link` | fixed | 1.98  | 0.00  |  0.46 |

Defined in `urdf/sensors/my_boat_sensors.xacro`, instantiated by the
`my_boat_gazebo` macro. (`base_link` is at the COM.) TF hierarchy:
`base_link → {imu_link, gps_link, lidar_link}`.

## 2. Sensors

| Sensor | gz type     | rate   | ROS msg                  | Provides                                  |
|--------|-------------|-------:|--------------------------|-------------------------------------------|
| IMU    | `imu`       | 100 Hz | `sensor_msgs/Imu`        | orientation, angular_velocity, linear_acc |
| GPS    | `navsat`    | 10 Hz  | `sensor_msgs/NavSatFix`  | latitude, longitude, altitude             |
| LiDAR  | `gpu_lidar` | 10 Hz  | `sensor_msgs/PointCloud2`| 3D point cloud (GPU ray tracing)          |

IMU orientation reference frame is **ENU**.

**Noise (all zero for now).**
- IMU & GPS use the full Gaussian set: `mean, stddev, bias_mean, bias_stddev,
  dynamic_bias_stddev, dynamic_bias_correlation_time, precision` — IMU on
  `angular_velocity`/`linear_acceleration` {x,y,z}; GPS on `position_sensing`/
  `velocity_sensing` {horizontal, vertical}.
- LiDAR uses the gz lidar/ray noise model, whose schema (sdformat `lidar.sdf`)
  defines **only** `type, mean, stddev` — `bias_*`/`precision` are IMU/GPS-only
  and not valid for lidar, so those two ARE the full available set (both zero).

To make the sensors realistic later, raise these (e.g. IMU gyro `stddev`,
GPS `position_sensing/horizontal stddev`, LiDAR range `stddev`) to the datasheet.

### LSLiDAR C32 scan configuration (`type="gpu_lidar"`)
| Setting | Value |
|---------|-------|
| Horizontal FOV | 360° (`min −π`, `max +π`), 1800 samples (~0.2°) |
| Vertical | 32 beams, `min −16°`, `max +15°` (31° FOV) |
| Range | `min 0.3 m`, `max 130 m`, `resolution 0.01 m` |
| Update rate | 10 Hz |
| frame_id | `my_boat/lidar_link` (`<gz_frame_id>`) |

**Recommended C32 emulation params:** horizontal samples ≈ **1800–2000** at
10 Hz give the real sensor's ~0.18–0.2° azimuth resolution (raise `h_samples`
for denser clouds at a sim-speed cost). All scan params are macro arguments on
`my_boat_lidar` in `my_boat_sensors.xacro`, so they are trivially retunable.

## 3. Topics & bridge

Gazebo publishes on the model-namespaced topics; `config/sensors.yaml` bridges
them to the ROS names the task asks for:

| Gazebo topic            | ROS topic       | ROS type                 | GZ type                   |
|-------------------------|-----------------|--------------------------|---------------------------|
| `/my_boat/imu/data`     | `/imu/data`     | `sensor_msgs/Imu`        | `gz.msgs.IMU`             |
| `/my_boat/gps/fix`      | `/gps/fix`      | `sensor_msgs/NavSatFix`  | `gz.msgs.NavSat`          |
| `/my_boat/lidar/points` | `/lidar/points` | `sensor_msgs/PointCloud2`| `gz.msgs.PointCloudPacked`|

The spawn launch starts this bridge automatically (`bridge:=true`, default).
(A `gpu_lidar` publishes its cloud on `<topic>/points`, so the sensor `<topic>`
`/my_boat/lidar` yields the cloud on `/my_boat/lidar/points`.)

## 4. World requirements

The sensors need these **world** systems + a datum (already in VRX competition
worlds; added to `worlds/examples/my_boat_test.sdf` too):
```
gz-sim-sensors-system   gz-sim-imu-system   gz-sim-navsat-system
<spherical_coordinates> (EARTH_WGS84, ENU, lat/lon datum)  ← needed for GPS lat/lon
```
Without `<spherical_coordinates>` the GPS cannot convert ENU → lat/lon.

## 5. Run & verify

```bash
cd ~/kavinesh/gazebo_vfx/vrx_garden_ws
colcon build --packages-select my_boat_description my_boat_gazebo
source install/setup.bash

# world + boat (+ bridges)
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
ros2 launch my_boat_gazebo spawn_my_boat.launch.py
```

### IMU
```bash
ros2 topic echo /imu/data
```
- `orientation` is populated.
- `angular_velocity` changes when the vessel rotates (drive `T1`+ / `T2`− or steer T3).
- `linear_acceleration` changes during accel/decel (e.g. `z ≈ +9.8` at rest = gravity).

### GPS
```bash
ros2 topic echo /gps/fix
```
- `latitude` / `longitude` / `altitude` are populated.
- lat/lon change as the boat moves (command `T3` thrust and watch them drift).

### LiDAR
```bash
ros2 topic list | grep points          # /lidar/points exists
ros2 topic echo /lidar/points --once   # a PointCloud2 message
ros2 topic hz /lidar/points            # ~10 Hz
```
RViz (set **Fixed Frame** to `my_boat/base_link` or `my_boat/lidar_link`, add a
**PointCloud2** display on `/lidar/points`), verify:
- full **360°** coverage and **32** vertical layers,
- cloud originates at the mast position (x≈1.98, z≈0.46) and rotates with the boat,
- drop obstacles near the boat → they appear with sane ranges (0.3–130 m).

TF note: run a `robot_state_publisher` with the generated URDF (or the spawn's
pose publisher) so `my_boat/lidar_link` is in TF for RViz.

Quick checks without echoing full messages:
```bash
ros2 topic hz /imu/data        # ~100 Hz
ros2 topic hz /gps/fix         # ~10 Hz
ros2 topic hz /lidar/points    # ~10 Hz
gz topic -l | grep -E 'my_boat/(imu|gps|lidar)'   # Gazebo side present
```
