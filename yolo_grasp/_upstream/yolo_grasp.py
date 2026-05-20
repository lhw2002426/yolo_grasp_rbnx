#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""yolo_grasp.py — placeholder.

The real implementation lives at the deploy machine path
    /home/syswonder/lhw/detect_grasp/yolo_grasp.py
and was not copied into this workspace yet (TODO: 用户后补).

Functionally this is the **actual** grasp-pose estimator used by the
current pick pipeline. The graspnet-baseline `demo_ros2.py` under
`/Users/howenliu/lab/grasp/driver/graspnet/src/graspnet-baseline/` is
NOT in the live path — the `source ... graspnet/install/setup.bash`
in this script's runtime is purely to expose the `graspnet_msgs`
message package (GraspPose / GraspRequest / ObjectDetectionRequest /
PiperStatusMsg). The actual grasp-pose math is geometric / heuristic
on top of a YOLO-World detection, not the heavy 3D-graspnet model.

Expected runtime contract (mirrors what the legacy ROS service
clients in `skill/pick/pick.py` and `piper_moveit_control` see):

    Service in:  /yolo/detect_object       (graspnet_msgs/srv/ObjectDetectionRequest)
                 — actually owned by yoloe/object_detection_node.py;
                   yolo_grasp.py CALLS this as a client to localise
                   the requested object.
    Service in:  /graspnet/grasp_request   (graspnet_msgs/srv/GraspRequest)
                 — yolo_grasp.py OWNS this; on call it computes the
                   grasp pose from the YOLO 3D bbox / center and
                   returns a PoseStamped + gripper_width.
    Topic out:   /graspnet/grasps          (graspnet_msgs/msg/GraspPose)
                 — yolo_grasp.py also publishes the result here so
                   piper_moveit_control's C++ subscriber kicks in.

This placeholder is wired so that the migration plan in
`MIGRATION_PLAN.md` can be reviewed coherently before the real source
lands. Do NOT import or run this file as-is — it has no implementation.
"""
from __future__ import annotations

import sys


def main() -> int:
    print(
        "[yolo_grasp] placeholder — real implementation pending "
        "(copy from /home/syswonder/lhw/detect_grasp/yolo_grasp.py).",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
