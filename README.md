# yolo_grasp_rbnx

Robonix package for geometric grasp-pose estimation on the Piper +
Orbbec Dabai DCW grasp pipeline. Stage 4B of the migration.

> ⚠️  **PLACEHOLDER MODE.** The real grasp-pose math hasn't been
> copied into this workspace yet. Every `grasp_request` call
> currently returns `success=false` with a clear placeholder
> message. See [Placeholder mode](#placeholder-mode) for the cutover.

## What it does (when wired up)

Geometric / heuristic grasp-pose estimator on top of a YOLO-World 2D
detection. Pure CPU, no ML model. Owns
`robonix/service/perception/grasp_pose/*`. Three roles:

| role | surface | who talks to it |
|---|---|---|
| **Server (atlas MCP)** | `grasp_pose/grasp_request` | Pilot's LLM (new path) |
| **Server (legacy ROS)** | `/graspnet/grasp_request` (graspnet_msgs/srv/GraspRequest) | pick.py (Stage 6 will switch to MCP) |
| **Topic publisher** | `/graspnet/grasps` (graspnet_msgs/msg/GraspPose) | C++ piper_moveit_control subscriber |
| **Client (legacy ROS)** | `/yolo/detect_object` | yolo_world_rbnx (Stage 4A) |

When called without a `bbox_2d` / `object_center_3d` hint, the handler
calls `/yolo/detect_object` itself to localise the target. The C++
piper_moveit_control subscriber on `/graspnet/grasps` always gets a
fire-and-forget message on success even when the caller used the MCP
path — that way Stage 5 doesn't have to track which surface produced
the pose.

## Two surfaces, one estimator

```
       Pilot LLM           pick.py (Stage 6 cutover)
           │                       │
           ▼                       ▼
    atlas-routed MCP          ROS service
    grasp_request           /graspnet/grasp_request
           │                       │
           └──► _serve_grasp_request() ◄──┘
                          │
                          ▼
                   _estimate_grasp_pose()
                          │
                          ▼              ┌─► topic /graspnet/grasps
                   GraspPose response ───┤   (C++ moveit_control subscriber)
                                          └─► (also returned synchronously)
```

`_serve_grasp_request()` may internally call upstream
`/yolo/detect_object` when the caller doesn't provide a bbox.

## Architecture

```
yolo_grasp_rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── service/perception/grasp_pose/
│   │   ├── driver.v1.toml          # rpc, lifecycle/srv/Driver.srv
│   │   ├── grasp_request.v1.toml   # rpc/MCP, grasp/srv/GraspRequest.srv
│   │   └── grasps.v1.toml          # topic_out/ROS2, grasp/msg/GraspPose.msg
│   └── lib/grasp/
│       ├── srv/GraspRequest.srv    # codegen → GraspRequest_Request/_Response
│       └── msg/GraspPose.msg       # codegen → GraspPose dataclass
├── yolo_grasp/
│   ├── __init__.py
│   ├── main.py                     # robonix Service + rclpy thread
│   └── _upstream/
│       └── yolo_grasp.py           # ⚠️ PLACEHOLDER — see below
├── scripts/
│   ├── build.sh                    # colcon graspnet_msgs + rbnx codegen --mcp
│   └── start.sh                    # source overlays, exec yolo_grasp.main
└── src/
    └── graspnet_msgs/              # vendored (32 KB)
```

## Lifecycle

```
on_init ── parse cfg ──► atlas resolve detect_object endpoint (informational)
                       ──► spawn rclpy thread
                           (ROS service host
                            + /graspnet/grasps publisher
                            + /yolo/detect_object client)

on_deactivate ── stop rclpy thread.
```

Note: this package's `on_init` does not depend on `/yolo/detect_object`
being up — we wait up to 30s for the upstream service in the rclpy
thread, log a warning if it never appears, and let `grasp_request`
calls without a bbox fail cleanly afterwards. This keeps boot from
deadlocking when yolo_world_rbnx is still warming up.

## Placeholder mode

`yolo_grasp/main.py` ships with `IS_PLACEHOLDER = True`. While that
flag is True:

* Every `grasp_request` returns `success=false, message="...PLACEHOLDER..."`
* `/graspnet/grasps` topic is **not** published (we publish only on
  success, and the placeholder never succeeds)
* The atlas plumbing, ROS service host, ROS client all work — only
  the math is missing

### Cutover steps (when the real code arrives)

The real implementation is at
`/home/syswonder/lhw/detect_grasp/yolo_grasp.py` on the deploy machine.

1. Copy it to `yolo_grasp/_upstream/yolo_grasp.py` (overwrite the
   placeholder shipped here).

2. Inspect what it provides — typically a function like
   `compute_grasp_pose(object_name, bbox_2d, object_center_3d, ...)`
   returning a `(PoseStamped, gripper_width, score)` tuple.

3. In `yolo_grasp/main.py`:
   * Set `IS_PLACEHOLDER = False`.
   * Replace the body of `_estimate_grasp_pose` with calls into the
     upstream estimator. The expected output dict shape is
     documented in that function's docstring.
   * If the upstream uses a separate ROS node / topic-driven approach
     instead of a callable, just import + use whatever functions it
     exposes from `_upstream.yolo_grasp`. Keep the dict-return
     contract; the surface code doesn't care how the math is done.

4. Add config knobs to `package_manifest.yaml` for any tunable the
   upstream estimator exposes (approach distance, max gripper width,
   score threshold, etc.).

5. Drop the `_upstream/` namespace altogether if the math is small
   enough to inline. The directory is just a holding pen for the
   verbatim upstream file.

## Build / run

```bash
cd /Users/howenliu/lab/packages/yolo_grasp_rbnx
bash scripts/build.sh

cd /Users/howenliu/lab/piper_grasp_deploy
rbnx boot
```

## Verification (in order)

```bash
# 1. atlas-side: provider + capabilities visible
rbnx caps | grep yolo_grasp
# expect:
#   yolo_grasp  com.robonix.piper_grasp.yolo_grasp  ACTIVE
#     robonix/service/perception/grasp_pose/driver         (rpc/grpc)
#     robonix/service/perception/grasp_pose/grasp_request  (rpc/mcp)
#     robonix/service/perception/grasp_pose/grasps         (topic_out/ros2)

# 2. With IS_PLACEHOLDER=True, MCP path should return a clean failure:
rbnx ask "grasp the cup on the table"
# pilot calls grasp_request → expects success=false with placeholder message

# 3. Legacy ROS service shape — also returns the placeholder failure:
ros2 service call /graspnet/grasp_request \
    graspnet_msgs/srv/GraspRequest "{object_name: 'cup', retry: 0}"

# 4. After flipping IS_PLACEHOLDER=False (with real estimator wired):
ros2 topic echo /graspnet/grasps --once
# expect: a GraspPose with non-trivial target_pose + gripper_width > 0
```

## Failure modes

| symptom | cause | fix |
|---|---|---|
| `grasp_request` always returns "PLACEHOLDER" | Expected (default state) | Wire upstream estimator, see "Cutover steps" |
| `detect_object pre-call failed: service not advertised` | yolo_world_rbnx not active | Check `rbnx caps yolo_world`; ensure Stage 4A is up first |
| MCP path returns "ROS thread not initialized" | on_init not yet completed | rbnx boot reports the actual blocker; check the package log |
| `/graspnet/grasps` silent even on success | C++ subscriber not consuming, or `_grasps_pub` never created | Check rclpy thread stayed alive (look for "rclpy thread exited" in logs) |

## Coupling with neighbors

* **Upstream** yolo_world_rbnx (Stage 4A) — provides
  `/yolo/detect_object` and the atlas MCP `object_detect/detect_object`.
  yolo_grasp_rbnx calls it as a CLIENT.
* **Downstream** piper_moveit_rbnx (Stage 5) — owns the C++
  `piper_moveit_control` subscriber on `/graspnet/grasps`. Plus pick_skill_rbnx
  (Stage 6) will call `grasp_request` over MCP.

So the deploy ordering is:
```
yolo_world ── yolo_grasp ── piper_moveit ── pick_skill
```
