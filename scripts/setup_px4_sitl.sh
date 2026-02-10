#!/bin/bash

TARGET_FILE="PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"

echo "🔧 Patching PX4 SITL configuration for Offboard Control..."

if grep -q "COM_RCL_EXCEPT" "$TARGET_FILE"; then
    echo "✅ Configuration already present. Skipping."
else
    cat <<EOT >> "$TARGET_FILE"

# --- CUSTOM CONFIG BY ROS2 PROJECT ---
# Disable RC requirement
param set NAV_RCL_ACT 0
param set COM_RCL_EXCEPT 4
# Disable GCS requirement
param set NAV_DLL_ACT 0
param set COM_DLL_ACT 0
# Disable Arming checks
param set COM_ARM_CHECK 0
# Force Offboard logic
param set COM_OBL_ACT 0
# -------------------------------------
EOT
    echo "✅ Patch applied successfully!"
fi