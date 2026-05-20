#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""yolo_grasp_rbnx — geometric grasp-pose estimator service.

Owns ``robonix/service/perception/grasp_pose/*``. Pure CPU — no ML
model. Implements the geometric / heuristic logic the live pipeline
already uses (detect_grasp/yolo_grasp.py upstream): given a YOLO 3D
bbox + center, pick a grasp pose along the camera Z axis with a
fixed standard gripper width.

Two parallel surfaces, sharing one estimator function:

    1. Atlas-routed MCP   (the new path, what Pilot's LLM sees)
       robonix/service/perception/grasp_pose/grasp_request
       — input/output: GraspRequest_Request / _Response (codegen'd
         from capabilities/lib/grasp/srv/GraspRequest.srv)

    2. Legacy ROS service (compat path, what pick.py + the C++
       moveit_control subscriber still call/listen-to)
       /graspnet/grasp_request  (graspnet_msgs/srv/GraspRequest)
       + /graspnet/grasps topic  (graspnet_msgs/msg/GraspPose)

Plus a *third* role: this package CALLS yolo_world_rbnx as a CLIENT
(via ROS service /yolo/detect_object) when bbox_2d / object_center_3d
are not provided by the caller.

Lifecycle (per Robonix developer guide §5):
    on_init      — light-medium: parse cfg, atlas-resolve upstream
                   detect_object endpoint (informational; we still
                   call via ROS service for compat), spawn rclpy
                   thread (ROS service host + topic publisher +
                   detect_object client).
    on_deactivate — stop rclpy thread.

Placeholder note (2026-05-19):
    The real grasp-pose algorithm lives at
    `/home/syswonder/lhw/detect_grasp/yolo_grasp.py` on the deploy
    machine, not in this workspace yet. Until that file lands, this
    package ships a STUB estimator that:
      * registers all the right contracts and ROS topics so the
        pipeline plumbing can be tested;
      * returns success=false with a clear "placeholder" message
        on every grasp_request call;
      * is structured so dropping in the real implementation is a
        single function replace (see _estimate_grasp_pose below).

When the real yolo_grasp.py is available:
    1. Copy it to yolo_grasp/_upstream/yolo_grasp.py (overwrite
       the placeholder shipped here).
    2. Edit `_estimate_grasp_pose` in this file to forward to the
       upstream estimator function (or just inline the logic).
    3. Remove the IS_PLACEHOLDER guard.
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import ATLAS, Service, Ok, Err  # noqa: E402

logging.basicConfig(
    level=os.environ.get("YOLO_GRASP_LOG_LEVEL", "INFO"),
    format="[yolo_grasp] %(message)s",
)
log = logging.getLogger("yolo_grasp")

yolo_grasp = Service(
    id=os.environ.get("ROBONIX_CAPABILITY_ID", "yolo_grasp"),
    namespace="robonix/service/perception/grasp_pose",
)

# ── placeholder guard ────────────────────────────────────────────────────────
# Set to False when the real upstream yolo_grasp.py logic is wired into
# _estimate_grasp_pose. Until then, every grasp_request returns a
# clean "placeholder" failure instead of silently producing a bogus
# pose that the arm could try to execute.
IS_PLACEHOLDER = True

# ── shared state ────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_initialized = False
_resolved_cfg: Optional[dict[str, Any]] = None

_ros_node = None
_ros_thread: Optional[threading.Thread] = None
_ros_stop_evt = threading.Event()
_grasps_pub = None       # /graspnet/grasps publisher
_detect_client = None    # /yolo/detect_object service client


# ── atlas: resolve upstream detect_object (informational) ───────────────────
def _resolve_detect_object_endpoint() -> Optional[str]:
    """Query atlas for object_detect's MCP endpoint. We don't use the
    MCP path yet (Stage 6 will), but logging it is a useful sanity
    check that yolo_world_rbnx is up before we proceed."""
    cid = "robonix/service/perception/object_detect/detect_object"
    try:
        caps = ATLAS.find_capability(contract_id=cid, transport="mcp")
    except Exception as e:  # noqa: BLE001
        log.warning("atlas query %s failed: %s", cid, e)
        return None
    if not caps:
        log.warning(
            "atlas has no provider for %s — yolo_world_rbnx not active "
            "or not yet declared its MCP endpoint",
            cid,
        )
        return None
    try:
        ch = yolo_grasp.connect_capability(caps[0], cid, "mcp")
        ep = ch.endpoint
        try:
            ch.close()
        except Exception:  # noqa: BLE001
            pass
        log.info("atlas resolved %s @ %s (provider=%s)",
                 cid, ep, caps[0].provider_id)
        return ep
    except Exception as e:  # noqa: BLE001
        log.warning("atlas connect %s failed: %s", cid, e)
        return None


# ── grasp-pose estimator (PLACEHOLDER — replace with upstream logic) ────────
def _estimate_grasp_pose(
    object_name: str,
    bbox_2d: list[float],
    object_center_3d: list[float],
    retry: int,
) -> dict:
    """Stub: returns success=false with a placeholder message.

    When the real yolo_grasp.py lands, replace this body with the
    upstream estimator. The expected output dict keys are:
      {
        "success":       bool,
        "message":       str,
        "pose":          {position: {x, y, z},
                          orientation: {x, y, z, w}},
        "frame_id":      str,           # always "camera_color_optical_frame"
        "gripper_width": float,
        "score":         float,
      }
    """
    if IS_PLACEHOLDER:
        return {
            "success":       False,
            "message":       (
                "yolo_grasp_rbnx is in PLACEHOLDER mode — copy the real "
                "/home/syswonder/lhw/detect_grasp/yolo_grasp.py into "
                "yolo_grasp/_upstream/, port the math into "
                "_estimate_grasp_pose, then flip IS_PLACEHOLDER=False"),
            "pose":          {
                "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "frame_id":      "camera_color_optical_frame",
            "gripper_width": 0.0,
            "score":         0.0,
        }

    # === real estimator goes below this line ===
    # The upstream logic is roughly:
    #   1. If bbox_2d / object_center_3d empty, call /yolo/detect_object.
    #   2. Pick a grasp axis along camera +Z (top-down approach).
    #   3. Compute orientation from the camera-frame vertical, with
    #      yaw biased towards the bbox aspect ratio.
    #   4. gripper_width = clamp(bbox_2d.short_side * z_depth * fx_scale,
    #                            min_w, max_w)
    #   5. score = combination of bbox_conf + depth_validity + ...
    raise NotImplementedError(
        "real estimator not wired; see _upstream/yolo_grasp.py")


# ── ROS bring-up (background thread) ────────────────────────────────────────
def _ros_thread_main() -> None:
    global _ros_node, _grasps_pub, _detect_client

    import rclpy                                              # noqa: E402
    from rclpy.node import Node                               # noqa: E402
    from graspnet_msgs.srv import GraspRequest                # noqa: E402
    from graspnet_msgs.srv import ObjectDetectionRequest      # noqa: E402
    from graspnet_msgs.msg import GraspPose as RosGraspPose   # noqa: E402

    rclpy.init(args=None)
    node = Node("yolo_grasp_node")
    _ros_node = node

    # /graspnet/grasps publisher — C++ piper_moveit_control subscribes here.
    _grasps_pub = node.create_publisher(RosGraspPose, "/graspnet/grasps", 10)

    # /yolo/detect_object client — used when caller didn't supply a bbox.
    _detect_client = node.create_client(
        ObjectDetectionRequest, "/yolo/detect_object")
    log.info("waiting for /yolo/detect_object service (yolo_world_rbnx)…")
    waited = 0.0
    while not _detect_client.wait_for_service(timeout_sec=0.5):
        if _ros_stop_evt.is_set():
            return
        waited += 0.5
        if waited >= 30.0:
            log.warning(
                "/yolo/detect_object not up after 30s — continuing anyway; "
                "grasp_request calls without bbox will fail until it appears")
            break

    # /graspnet/grasp_request service host.
    def _grasp_request_cb(req, resp):
        result = _serve_grasp_request(
            object_name      = req.object_name,
            bbox_2d          = list(req.bbox_2d) if req.bbox_2d else [],
            object_center_3d = list(req.object_center_3d) if req.object_center_3d else [],
            retry            = int(req.retry),
        )
        # Pack into ROS response.
        resp.success       = bool(result["success"])
        resp.message       = str(result["message"])
        resp.gripper_width = float(result["gripper_width"])
        resp.score         = float(result["score"])
        # PoseStamped in result["pose"]:
        from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion  # noqa
        ps = PoseStamped()
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.header.frame_id = result["frame_id"]
        p = result["pose"]
        ps.pose = Pose(
            position=Point(
                x=float(p["position"]["x"]),
                y=float(p["position"]["y"]),
                z=float(p["position"]["z"])),
            orientation=Quaternion(
                x=float(p["orientation"]["x"]),
                y=float(p["orientation"]["y"]),
                z=float(p["orientation"]["z"]),
                w=float(p["orientation"]["w"])),
        )
        resp.grasp_pose = ps

        # Fire-and-forget topic publish — this is what the legacy C++
        # subscriber consumes. We publish even on success=false so
        # subscribers don't sit waiting forever, but a placeholder
        # response will have all-zero pose + 0 width.
        if result["success"]:
            gp = RosGraspPose()
            gp.target_pose = ps
            gp.gripper_width = float(result["gripper_width"])
            _grasps_pub.publish(gp)
        return resp
    node.create_service(GraspRequest, "/graspnet/grasp_request", _grasp_request_cb)
    log.info("ROS service up: /graspnet/grasp_request")
    log.info("ROS publisher up: /graspnet/grasps")

    while not _ros_stop_evt.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    log.info("rclpy thread exited")


def _call_detect_object(object_name: str, timeout_s: float = 5.0) -> dict:
    """Synchronous client call to /yolo/detect_object (the legacy ROS
    surface owned by yolo_world_rbnx). Returns the response as a dict
    with the same keys yolo_world's _detect_object() exposes."""
    from graspnet_msgs.srv import ObjectDetectionRequest  # noqa: E402
    if _detect_client is None:
        return {"success": False,
                "message": "ROS thread not initialized",
                "bbox_2d": [], "object_center_3d": [], "confidence": 0.0}
    if not _detect_client.service_is_ready():
        return {"success": False,
                "message": "/yolo/detect_object service not advertised",
                "bbox_2d": [], "object_center_3d": [], "confidence": 0.0}
    req = ObjectDetectionRequest.Request()
    req.object_name = object_name
    fut = _detect_client.call_async(req)
    deadline = time.monotonic() + timeout_s
    while not fut.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not fut.done():
        return {"success": False,
                "message": f"/yolo/detect_object call timed out after {timeout_s}s",
                "bbox_2d": [], "object_center_3d": [], "confidence": 0.0}
    resp = fut.result()
    return {
        "success":          bool(resp.success),
        "message":          str(resp.message),
        "bbox_2d":          list(resp.bbox_2d),
        "object_center_3d": list(resp.object_center_3d),
        "confidence":       float(resp.confidence),
    }


def _serve_grasp_request(*, object_name, bbox_2d, object_center_3d, retry):
    """Shared handler for both surfaces.

    If bbox_2d / object_center_3d are missing, we call the upstream
    yolo_world detect_object service first (legacy ROS path; will swap
    to MCP in Stage 6).
    """
    if not bbox_2d or not object_center_3d:
        det = _call_detect_object(object_name)
        if not det["success"]:
            return {
                "success": False,
                "message": f"detect_object pre-call failed: {det['message']}",
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
                "frame_id": "camera_color_optical_frame",
                "gripper_width": 0.0,
                "score": 0.0,
            }
        bbox_2d          = det["bbox_2d"]
        object_center_3d = det["object_center_3d"]

    return _estimate_grasp_pose(object_name, bbox_2d, object_center_3d, retry)


# ── lifecycle ───────────────────────────────────────────────────────────────
@yolo_grasp.on_init
def init(cfg):
    """Driver(CMD_INIT). Light-medium:
      1. parse cfg
      2. atlas-resolve detect_object endpoint (informational)
      3. spawn rclpy thread
    """
    global _initialized, _resolved_cfg
    with _state_lock:
        if _initialized:
            return Ok()

    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")
    _resolved_cfg = cfg

    if IS_PLACEHOLDER:
        log.warning(
            "===========================================================\n"
            "[yolo_grasp] running in PLACEHOLDER mode!\n"
            "All grasp_request calls return success=false. Drop in the\n"
            "real upstream estimator and flip IS_PLACEHOLDER=False to\n"
            "go live. See yolo_grasp/main.py header for instructions.\n"
            "===========================================================")

    # Informational: log which provider owns detect_object on atlas.
    _resolve_detect_object_endpoint()

    # Spawn rclpy thread. We don't sentinel-wait on /graspnet/grasp_request
    # being advertised — the rclpy thread itself is what advertises it,
    # and on_init returning Ok() is what triggers atlas to mark us ACTIVE.
    global _ros_thread
    _ros_stop_evt.clear()
    _ros_thread = threading.Thread(
        target=_ros_thread_main,
        name="yolo_grasp-ros",
        daemon=True,
    )
    _ros_thread.start()
    time.sleep(0.5)  # let create_service / create_publisher land

    with _state_lock:
        _initialized = True
    log.info("init complete: grasp_request MCP + /graspnet/grasp_request live")
    return Ok()


@yolo_grasp.on_deactivate
def deactivate():
    log.info("CMD_DEACTIVATE: stopping rclpy thread")
    _ros_stop_evt.set()
    if _ros_thread is not None:
        _ros_thread.join(timeout=5.0)
    with _state_lock:
        global _initialized
        _initialized = False
    return Ok()


# ── atlas-routed MCP handler (Pilot's view) ─────────────────────────────────
# Import top-level Request/Response from the package-local IDL, plus
# the nested geometry_msgs / std_msgs / builtin_interfaces types we
# need to instantiate. The `_mcp` suffix is codegen's convention:
# `{ros_package}_mcp.py` per ROS package (see
# robonix-codegen/src/codegen/mcp_python_gen.rs:4).
from grasp_mcp import (  # noqa: E402  pylint: disable=wrong-import-position
    GraspRequest_Request, GraspRequest_Response,
)
from geometry_msgs_mcp import (  # noqa: E402
    PoseStamped, Pose, Point, Quaternion,
)
from std_msgs_mcp import Header  # noqa: E402
from builtin_interfaces_mcp import Time  # noqa: E402


@yolo_grasp.mcp("robonix/service/perception/grasp_pose/grasp_request")
def grasp_request(req: GraspRequest_Request) -> GraspRequest_Response:
    """Compute a grasp pose for `req.object_name` (open vocab via
    upstream YOLOE)."""
    result = _serve_grasp_request(
        object_name      = req.object_name,
        bbox_2d          = list(req.bbox_2d) if req.bbox_2d else [],
        object_center_3d = list(req.object_center_3d) if req.object_center_3d else [],
        retry            = int(req.retry),
    )
    p = result["pose"]
    pose_stamped = PoseStamped(
        header=Header(
            stamp=Time(sec=0, nanosec=0),
            frame_id=result["frame_id"],
        ),
        pose=Pose(
            position=Point(
                x=float(p["position"]["x"]),
                y=float(p["position"]["y"]),
                z=float(p["position"]["z"])),
            orientation=Quaternion(
                x=float(p["orientation"]["x"]),
                y=float(p["orientation"]["y"]),
                z=float(p["orientation"]["z"]),
                w=float(p["orientation"]["w"])),
        ),
    )
    # Also publish to legacy topic so the C++ piper_moveit_control
    # subscriber kicks in even if the MCP path is the one being used.
    if result["success"] and _grasps_pub is not None:
        try:
            from graspnet_msgs.msg import GraspPose as RosGraspPose  # noqa: E402
            from geometry_msgs.msg import (
                PoseStamped as RosPoseStamped, Pose as RosPose,
                Point as RosPoint, Quaternion as RosQuaternion)  # noqa: E402
            ros_ps = RosPoseStamped()
            if _ros_node is not None:
                ros_ps.header.stamp = _ros_node.get_clock().now().to_msg()
            ros_ps.header.frame_id = result["frame_id"]
            ros_ps.pose = RosPose(
                position=RosPoint(
                    x=float(p["position"]["x"]),
                    y=float(p["position"]["y"]),
                    z=float(p["position"]["z"])),
                orientation=RosQuaternion(
                    x=float(p["orientation"]["x"]),
                    y=float(p["orientation"]["y"]),
                    z=float(p["orientation"]["z"]),
                    w=float(p["orientation"]["w"])),
            )
            gp = RosGraspPose()
            gp.target_pose = ros_ps
            gp.gripper_width = float(result["gripper_width"])
            _grasps_pub.publish(gp)
        except Exception as e:  # noqa: BLE001
            log.warning("legacy /graspnet/grasps publish failed: %s", e)

    return GraspRequest_Response(
        grasp_pose    = pose_stamped,
        gripper_width = float(result["gripper_width"]),
        score         = float(result["score"]),
        success       = bool(result["success"]),
        message       = str(result["message"]),
    )


def main() -> int:
    import signal
    def _on_signal(sig, _frame):
        log.info("signal %d — shutting down", sig)
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT,  _on_signal)
    try:
        yolo_grasp.run()
    finally:
        _ros_stop_evt.set()
        if _ros_thread is not None:
            _ros_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
