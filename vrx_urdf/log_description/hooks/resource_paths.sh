#!/usr/bin/env bash
# Hook script: appends this package's models directory to GZ_SIM_RESOURCE_PATH
# so that Gazebo Garden can locate the floating_log SDF model at runtime.
# Pattern mirrors the approach used by VRX's wamv_description package.

PACKAGE_SHARE_DIR="$(ros2 pkg prefix --share log_description 2>/dev/null)"

if [ -n "${PACKAGE_SHARE_DIR}" ]; then
  ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "${PACKAGE_SHARE_DIR}/models"
fi
