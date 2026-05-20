#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase. Two steps:
#   1. colcon build vendored graspnet_msgs (msg + srv Python bindings
#      for the legacy ROS surface).
#   2. rbnx codegen --mcp:
#        * atlas_pb2 / atlas_pb2_grpc                   (Service runtime)
#        * grasp_mcp.py                                  (GraspRequest_Request/_Response,
#                                                         GraspPose dataclass)
#        * geometry_msgs_mcp.py / std_msgs_mcp.py /
#          builtin_interfaces_mcp.py                    (PoseStamped + nested)
#
# We don't need any ML deps — yolo_grasp_rbnx is pure CPU geometry.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[yolo_grasp/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

ln -snf "$PKG/src/graspnet_msgs" "$PKG/rbnx-build/ws/src/graspnet_msgs"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[yolo_grasp/build] colcon build (graspnet_msgs)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install \
    --packages-select graspnet_msgs \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[yolo_grasp/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[yolo_grasp/build] done."
