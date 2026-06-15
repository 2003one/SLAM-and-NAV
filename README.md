# Autonomous TurtleBot3 — SLAM Mapping & Nav2 Navigation

Drive a TurtleBot3 around a Gazebo world **autonomously (no teleop)** to build a
map with SLAM, save that map, then have the robot **navigate it on its own** with
Nav2.

The repo includes `auto_nav`, a reactive explorer node that drives the robot
hands-free so SLAM can map the environment without you steering with the keyboard.

![Nav2 running on the saved TurtleBot3 map in RViz, with Gazebo on the left](nav2_rviz.png)

*Nav2 on the saved map — RViz shows the costmap, the planned path, and the
Navigation 2 panel, with the Gazebo simulation on the left.*

---

## Tested environment

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (`gz sim` 8.x) |
| Robot | TurtleBot3 **burger** |

> ⚠️ The Jazzy + Gazebo Harmonic combo behaves differently from the older
> Humble + Gazebo Classic tutorials. The biggest difference — the `/cmd_vel`
> message type — is covered in [Troubleshooting](#troubleshooting--the-movement-problem).

---

## Dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-teleop-twist-keyboard
```

## Environment setup (once)

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

## Build

```bash
git clone <this-repo-url> Auto
cd Auto/ros
colcon build --symlink-install
source install/setup.bash
```

This package is named `ros`, and the explorer runs as `ros2 run ros auto_nav`.

---

## How `auto_nav` works (driving without teleop)

`auto_nav` is a **continuous proportional controller**, not a bang-bang state
machine. That distinction is what makes the motion smooth and car-like instead of
jittery:

- **Speed** scales with how clear the path ahead is — full speed when open, easing
  off as obstacles approach.
- **Steering** is a smooth function of left/right clearance, so the robot stays
  centered in corridors and gently curves toward open space.
- A **reverse-and-turn recovery** triggers only at a true dead end.
- It publishes **`geometry_msgs/msg/TwistStamped`** on `/cmd_vel` (required on
  Jazzy — see Troubleshooting).

The result: the robot wanders the whole world on its own, which is exactly what
SLAM needs to fill in the map — no keyboard driving required.

---

## Usage

### Phase 1 — Build and save a map (autonomous, no teleop)

Run each command in its **own terminal**, in order. Wait for each to settle before
starting the next.

```bash
# Terminal 1 — simulator (wait for the Gazebo window + robot to appear)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
```bash
# Terminal 2 — SLAM (builds the map live as the robot drives)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
```bash
# Terminal 3 — RViz (NOTE: use_sim_time stops the double-robot flicker)
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```
In RViz: set **Fixed Frame → `map`** and the Map display **Durability → Transient Local**.

```bash
# Terminal 4 — the autonomous explorer (drives itself)
source ~/Auto/ros/install/setup.bash
ros2 run ros auto_nav --ros-args -p use_sim_time:=true
```

Let it drive until the map looks **complete** in RViz — solid closed walls, white
interior, no big grey gaps. Then save it:

```bash
# Terminal 5 — save the map
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

> `map_saver_cli` is a **one-shot snapshot** — it grabs whatever `/map` looks like
> at that instant and exits. Only save once the map looks finished, and re-run it
> any time to overwrite with a fuller version.

This produces `~/maps/my_map.pgm` and `~/maps/my_map.yaml`.

### Phase 2 — Navigate the saved map (Nav2)

```bash
# Terminal 1 — simulator
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
```bash
# Terminal 2 — Nav2 with the saved map AND the patched params (see Troubleshooting)
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$HOME/maps/my_map.yaml \
  params_file:=$HOME/nav2_params.yaml
```
```bash
# Terminal 3 — RViz
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

In RViz:
1. **2D Pose Estimate** — click where the robot actually sits in Gazebo and drag in
   its facing direction. The laser scan should snap onto the walls.
2. **Nav2 Goal** — click a destination. The robot plans a path over the saved map
   and drives there on its own.

---

## Troubleshooting — the movement problem

This is the consolidated list of every issue that stopped the robot from moving,
and the fix for each. They are ordered roughly by how often they bite.

### 1. Robot won't move under Nav2 — `/cmd_vel` type mismatch (the big one)

**Symptom:** Nav2 accepts the goal and a path appears in RViz, but the robot never
moves. `ros2 topic echo /cmd_vel` prints nothing.

**Cause:** On **Jazzy, Nav2 publishes plain `Twist` by default**, but the Gazebo
bridge (`ros_gz_bridge`) subscribes to **`TwistStamped`**. Same topic name,
different message type → every command is silently dropped.

**Diagnose:**
```bash
ros2 topic info /cmd_vel -v
```
If the publishers say `geometry_msgs/msg/Twist` and the subscriber says
`geometry_msgs/msg/TwistStamped` (different type hashes), that's it.

**Fix:** make Nav2 publish stamped. Copy the default params and enable
`enable_stamped_cmd_vel` on every node that touches `/cmd_vel`:

```bash
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml ~/nav2_params.yaml

python3 - <<'EOF'
import yaml, os
path = os.path.expanduser('~/nav2_params.yaml')
with open(path) as f:
    d = yaml.safe_load(f)
for n in ['collision_monitor', 'docking_server', 'controller_server',
          'behavior_server', 'velocity_smoother']:
    if isinstance(d.get(n), dict) and 'ros__parameters' in d[n]:
        d[n]['ros__parameters']['enable_stamped_cmd_vel'] = True
        print('set for', n)
with open(path, 'w') as f:
    yaml.safe_dump(d, f, default_flow_style=False, sort_keys=False)
EOF
```

Then **fully restart Nav2** with `params_file:=$HOME/nav2_params.yaml` (see Phase 2).
Verify the fix with `ros2 topic info /cmd_vel -v` — publisher and subscriber should
both now say `TwistStamped` with matching type hashes.

The same mismatch affects **teleop** and any custom node. For teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link
```
For the `auto_nav` node, it already publishes `TwistStamped`.

### 2. Config changes "don't apply" — Nav2 was never actually restarted

**Symptom:** You edit params, but `ros2 topic info /cmd_vel -v` shows the old
behavior. The **node GIDs are identical** to before.

**Cause:** Nav2 reads its params **only at launch**. `Ctrl-C` sometimes leaves
nodes alive, so the old instance keeps running.

**Fix:** kill it hard, confirm it's gone, then relaunch.
```bash
pkill -9 -f nav2
pkill -9 -f collision_monitor
pkill -9 -f docking_server
ros2 node list      # confirm those nodes are gone before relaunching
```

### 3. Two robots / flickering in RViz

**Symptom:** Two robot models, one steady and one jittering, both flickering.

**Cause:** RViz launched **without sim time** while everything else uses it, so its
TF lookups flick in and out as timestamps mismatch.

**Fix:** launch RViz with sim time:
```bash
rviz2 -d <...> --ros-args -p use_sim_time:=true
```
Rule of thumb: in simulation, **every** node needs `use_sim_time:=true`.

### 4. Goal accepted, path drawn, but still no motion — inactive lifecycle nodes

**Symptom:** Path forms, `/cmd_vel` is silent. `ros2 lifecycle get /bt_navigator`
returns `inactive`.

**Cause:** Nav2's lifecycle manager didn't finish activating the stack on startup.

**Fix:** check all nodes and activate any that are inactive.
```bash
for n in controller_server smoother_server planner_server behavior_server \
         velocity_smoother collision_monitor waypoint_follower bt_navigator; do
  echo -n "$n: "; ros2 lifecycle get /$n
done

# activate any that are not 'active'
ros2 lifecycle set /bt_navigator activate
```
If the stack repeatedly fails to autostart, the cleanest long-term fix is to ensure
the params file is valid (a corrupted edit can stall activation), then relaunch so
the lifecycle manager brings everything up automatically.

### 5. Saved map is incomplete

**Symptom:** The saved `.pgm` is much smaller than the world.

**Cause:** `map_saver_cli` was run before the robot had driven everywhere — it only
snapshots what SLAM has seen so far.

**Fix:** keep SLAM and `auto_nav` running, let the robot cover the whole world
(watch the grey fill in), then re-run `map_saver_cli`.

### 6. Map invisible in RViz

**Fix:** Fixed Frame = `map`, and Map display **Durability Policy = Transient Local**.

---

## Quick reference

| Symptom | Fix |
|---|---|
| Robot won't move under Nav2 | `enable_stamped_cmd_vel: true` + restart Nav2 with `params_file` |
| Edits don't apply | `pkill -9 -f nav2`, confirm gone, relaunch |
| Two robots / flicker | add `--ros-args -p use_sim_time:=true` to RViz |
| Path drawn but no motion | `ros2 lifecycle set /bt_navigator activate` |
| Map too small | drive more, then re-run `map_saver_cli` |
| Map not visible | Fixed Frame `map`, Durability Transient Local |
| Teleop won't move robot | `-p stamped:=true -p frame_id:=base_link` |

---

## Pipeline at a glance

```
auto_nav (drives)  ─►  SLAM (builds map)  ─►  map_saver (saves map)
                                                     │
                                                     ▼
                              Nav2 (loads map, localizes, plans, drives autonomously)
```

**The core lesson:** on ROS 2 Jazzy with Gazebo Harmonic, the velocity command
topic uses `TwistStamped`, not `Twist`. Aligning every publisher and subscriber to
that type is what makes the robot actually move.
# SLAM-and-NAV
