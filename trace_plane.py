# trace_plane.py
#
# SO-ARM100 drawing plane simulation in MuJoCo.
#
# Loads a drawing exported from the p5 sketch (drawing.json) and drives
# the arm along the strokes using damped-least-squares IK. The LED-tip
# trail is visualized live in the viewer as red capsule segments so you
# can watch the drawing materialize on the (cyan-outlined) plane.
#
# Requires "led_tip" site in so_arm100.xml (inside Fixed_Jaw body):
#   <site name="led_tip" pos="0 0 0.05" size="0.001"/>
#
# USAGE:
#   mjpython trace_plane.py                   — run built-in test shape
#   mjpython trace_plane.py drawing.json      — load a p5 drawing
#
# CONTROLS:
#   SPACE  — run/replay the drawing (clears the old trail)
#   0      — workspace sweep

import json
import os
import sys
import time
from dataclasses import dataclass

import arm_executor
import mujoco
import mujoco.viewer
import numpy as np
import serial


# ============================================================================
# TYPE ALIASES
# ============================================================================

Point    = dict[str, float]   # {"x": float, "y": float} — normalized screen point
Stroke   = list[Point]        # ordered points in one pen-down stroke
Drawing  = list[Stroke]       # all strokes in a user's drawing
Waypoint = dict[str, float]   # {"x", "y", "z", "pen_down"} — 3D robot target
Path     = list[Waypoint]     # full ordered list of 3D targets for the arm


# ============================================================================
# CONFIGURATION
# ============================================================================

# --- Paths ---
SCENE_XML = os.path.join(os.path.dirname(__file__), "scene.xml")

# --- Real arm serial bridge ---
RUN_REAL_ARM = True   # set False for simulation only
ARM_PORT     = "/dev/cu.usbmodem1101"
ARM_BAUD     = 115200

# --- Home pose ---
HOME_QPOS = [-0.0576, -2.03, 0.837, 1.08, 0.0837, 0.0]

# --- IK ---
IK_DT        = 0.05
IK_DAMPING   = 0.01
IK_THRESHOLD = 0.005
IK_MAX_STEPS = 3000

# --- Drawing plane ---
HALF           = 0.1
DRAW_SCALE     = 2.0
PLANE_DISTANCE = 0.05  # metres from LED tip to plane surface

# --- Visualization ---
TRAIL_RADIUS  = 0.0008
TRAIL_MIN_SEG = 0.002
TRAIL_RGBA    = (1.0, 0.15, 0.15, 1.0)
PLANE_RGBA    = (0.2, 0.8, 1.0, 1.0)
PLANE_RADIUS  = 0.0007

# --- Timing ---
DRAW_SLEEP = 0.004
MOVE_SLEEP = 0.001
REAL_ARM_DRAW_SETTLE = 0.8
REAL_ARM_MOVE_SETTLE = 1.0

# --- Key codes ---
GLFW_KEY_SPACE = 32
GLFW_KEY_0     = 48


# ============================================================================
# STATE
# ============================================================================

@dataclass
class AppState:
    run_requested:   bool = False
    sweep_requested: bool = False


# Single-shot warning flag for when user_scn fills up on long drawings.
trail_full_warned: bool = False


# ============================================================================
# LOAD DRAWING
# ============================================================================

def load_drawing(json_path: str) -> Drawing:
    """
    Load strokes from a p5 export JSON file.

    The JSON's top-level "strokes" key holds a list of strokes; each stroke
    is a list of normalized {"x", "y"} points in 0–1 screen space.

    Args:
        json_path: filesystem path to the drawing JSON
    Returns:
        Drawing — the parsed list of strokes.
    """
    with open(json_path) as f:
        data = json.load(f)
    strokes: Drawing = data["strokes"]
    total_pts = sum(len(s) for s in strokes)
    print(f"Loaded {len(strokes)} strokes, {total_pts} points from {json_path}")
    return strokes


def test_drawing() -> Drawing:
    """
    Built-in test drawing used when no JSON path is supplied.

    Returns:
        Drawing — a triangle outline plus a single dot in the centre.
    """
    print("Using built-in test drawing (triangle)")
    return [
        # Triangle
        [
            {"x": 0.5, "y": 0.2},
            {"x": 0.8, "y": 0.8},
            {"x": 0.2, "y": 0.8},
            {"x": 0.5, "y": 0.2},
        ],
        # Dot in the middle
        [
            {"x": 0.5, "y": 0.55},
        ],
    ]


def strokes_to_path(strokes: Drawing, plane_center: np.ndarray, half: float) -> Path:
    """
    Convert normalized 2D strokes to 3D path points on the drawing plane.

    Coordinate mapping:

        Screen (p5)        Robot world
          x: 0→1    →    X: left → right on plane
          y: 0→1    →    Z: bottom → top (flipped: screen-Y down, world-Z up)
          (none)     →    Y: fixed at plane depth

    Between strokes pen_down=False (arm lifts to next stroke's start);
    within a stroke pen_down=True (arm draws).

    Args:
        strokes: list of strokes (normalized 0–1 points)
        plane_center: 3D centre of the drawing plane in world coords
        half: plane half-size in metres
    Returns:
        Path — list of waypoint dicts with x/y/z/pen_down keys.
    """
    path: Path = []
    for stroke in strokes:
        for i, pt in enumerate(stroke):
            x_3d = plane_center[0] + (pt["x"] - 0.5) * 2 * half
            z_3d = plane_center[2] + (0.5 - pt["y"]) * 2 * half
            y_3d = plane_center[1]

            path.append({
                "x": x_3d,
                "y": y_3d,
                "z": z_3d,
                "pen_down": (i > 0),   # first point of a stroke = pen up move
            })
    return path


# ============================================================================
# VISUALIZATION — append custom geoms to viewer.user_scn
# ============================================================================
#
# The passive viewer exposes a writable MjvScene as `viewer.user_scn`. Each
# frame, MuJoCo renders the geoms we've stuffed into it in addition to the
# simulation geoms. We use capsule "connectors" (mjv_connector) to draw:
#   • the plane outline (4 cyan edges)  — static reference frame
#   • the LED-tip trail  (red segments) — updated live as the arm moves
#
# user_scn has a fixed maxgeom, so every add is capacity-checked.

def add_capsule_segment(scn, p1, p2, radius: float, rgba) -> bool:
    """
    Append a capsule between p1 and p2 to the user scene.

    Args:
        scn: MjvScene to add into (typically viewer.user_scn)
        p1: 3D start point
        p2: 3D end point
        radius: capsule radius in metres
        rgba: (r, g, b, a) tuple
    Returns:
        True if the capsule was added, False if scn was already at maxgeom.
    """
    if scn.ngeom >= scn.maxgeom:
        return False
    geom = scn.geoms[scn.ngeom]
    # Initialize the geom slot (required before mjv_connector populates it).
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=np.zeros(3),
        pos=np.zeros(3),
        mat=np.eye(3).flatten(),
        rgba=np.array(rgba, dtype=np.float32),
    )
    # mjv_connector sets pos/mat/size so the capsule spans from p1 to p2.
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        radius,
        np.asarray(p1, dtype=np.float64),
        np.asarray(p2, dtype=np.float64),
    )
    scn.ngeom += 1
    return True


def draw_plane_outline(scn, center: np.ndarray, half: float) -> None:
    """
    Draw the rectangular plane frame in the user scene.

    Adds four capsule edges forming an XZ rectangle at center[1].

    Args:
        scn: MjvScene to add into
        center: 3D plane centre
        half: plane half-size in metres
    """
    cx, cy, cz = center
    corners = [
        np.array([cx - half, cy, cz - half]),
        np.array([cx + half, cy, cz - half]),
        np.array([cx + half, cy, cz + half]),
        np.array([cx - half, cy, cz + half]),
    ]
    for i in range(4):
        add_capsule_segment(scn, corners[i], corners[(i + 1) % 4],
                            radius=PLANE_RADIUS, rgba=PLANE_RGBA)


# ============================================================================
# IK + EXECUTION
# ============================================================================

def move_to_target(model, data, viewer, led_site_id: int,
                   target: np.ndarray, label: str, pen_down: bool = True) -> bool:
    """
    Run damped-least-squares IK until the LED tip reaches `target`.

    For a site position error e = target - current, the joint update is:
        dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
    This handles near-singular configurations gracefully (unlike the raw
    pseudoinverse) at the cost of slightly biased motion near singularities.

    Each iteration:
      1. Compute the 3×6 position Jacobian of the led_tip site.
      2. Solve the damped system for dq.
      3. Integrate qpos by dq * IK_DT, clip into joint limits.
      4. mj_kinematics + viewer.sync to redraw.
      5. If pen is down, append a red capsule to the trail.

    Args:
        model: MjModel
        data: MjData
        viewer: passive viewer (used for user_scn + sync)
        led_site_id: id of the led_tip site
        target: 3D world-frame target point
        label: short string used in console progress logs
        pen_down: if True, draw a red trail; if False, lift the pen
    Returns:
        True if converged within IK_THRESHOLD, False if max steps exhausted.
    """
    global trail_full_warned

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    # Anchor of the next trail segment — advances as the tip moves.
    last_trail_pos = data.site_xpos[led_site_id].copy()

    for step in range(IK_MAX_STEPS):
        current = data.site_xpos[led_site_id].copy()
        error   = target - current
        dist    = float(np.linalg.norm(error))

        if step % 500 == 0:
            print(f"  [{label}] step {step:4d} | err: {dist*100:.2f} cm | "
                  f"pen: {'DOWN' if pen_down else 'UP'}")

        if dist < IK_THRESHOLD:
            # Close out the trail to the exact stopping point so the
            # line doesn't fall a millimetre short of the waypoint.
            if pen_down and np.linalg.norm(current - last_trail_pos) > 1e-6:
                if not add_capsule_segment(viewer.user_scn, last_trail_pos,
                                           current, TRAIL_RADIUS, TRAIL_RGBA):
                    if not trail_full_warned:
                        print("  [trail] user_scn full — further segments dropped")
                        trail_full_warned = True
            return True

        # --- Jacobian at the LED site (position rows only) ---
        mujoco.mj_jacSite(model, data, jacp, jacr, led_site_id)
        J = jacp[:, :6]   # 3×6 — first 6 dof are the arm joints

        # --- Damped least-squares solve for dq ---
        JJT    = J @ J.T
        damped = JJT + (IK_DAMPING ** 2) * np.eye(3)
        dq     = J.T @ np.linalg.solve(damped, error)

        # --- Integrate then clip to joint limits ---
        data.qpos[:6] += dq * IK_DT
        for i in range(6):
            lo, hi = model.jnt_range[i]
            data.qpos[i] = float(np.clip(data.qpos[i], lo, hi))

        # Mirror qpos into ctrl so position actuators hold the pose
        # when the viewer steps dynamics internally.
        data.ctrl[:6] = data.qpos[:6]
        mujoco.mj_kinematics(model, data)

        # --- Trail update ---
        # Only emit a new capsule once the tip has travelled > TRAIL_MIN_SEG.
        if pen_down:
            new_pos = data.site_xpos[led_site_id].copy()
            if np.linalg.norm(new_pos - last_trail_pos) > TRAIL_MIN_SEG:
                if not add_capsule_segment(viewer.user_scn, last_trail_pos,
                                           new_pos, TRAIL_RADIUS, TRAIL_RGBA):
                    if not trail_full_warned:
                        print("  [trail] user_scn full — further segments dropped")
                        trail_full_warned = True
                last_trail_pos = new_pos

        viewer.sync()
        time.sleep(DRAW_SLEEP if pen_down else MOVE_SLEEP)

    print(f"  [{label}] max steps | err: {dist*100:.2f} cm")
    return False


def execute_path(model, data, viewer, led_site_id: int,
                 path_points: Path, arm_ser=None) -> None:
    """
    Drive the arm through every waypoint of a path in order.

    Args:
        model: MjModel
        data: MjData
        viewer: passive viewer
        led_site_id: id of the led_tip site
        path_points: list of waypoints to visit
        arm_ser: optional open serial connection to the real arm
    """
    current_counts = None
    for i, pt in enumerate(path_points):
        target = np.array([pt["x"], pt["y"], pt["z"]])
        label  = f"pt{i+1}/{len(path_points)}"
        move_to_target(model, data, viewer, led_site_id,
                       target, label, pen_down=pt["pen_down"])
        if arm_ser is not None:
            current_counts = arm_executor.send_joint_angles(
                arm_ser,
                data.qpos[:6].tolist(),
                pen_down=pt["pen_down"],
                current_counts=current_counts,
            )
            time.sleep(REAL_ARM_DRAW_SETTLE if pt["pen_down"] else REAL_ARM_MOVE_SETTLE)


def sweep_workspace(model, data, viewer, led_site_id: int) -> None:
    """
    Estimate the LED tip's reachable volume by random joint sampling.

    Prints the X/Y/Z bounds reached over 2000 random configurations,
    then restores the previous joint state.

    Args:
        model: MjModel
        data: MjData
        viewer: passive viewer (synced after restore)
        led_site_id: id of the led_tip site
    """
    print("\n=== Workspace sweep ===")
    reachable = []
    original_qpos = data.qpos[:6].copy()
    for _ in range(2000):
        for i in range(6):
            lo, hi = model.jnt_range[i]
            data.qpos[i] = np.random.uniform(lo, hi)
        mujoco.mj_kinematics(model, data)
        reachable.append(data.site_xpos[led_site_id].copy())
    reachable = np.array(reachable)
    print(f"  X: {reachable[:,0].min()*100:.1f} to {reachable[:,0].max()*100:.1f} cm")
    print(f"  Y: {reachable[:,1].min()*100:.1f} to {reachable[:,1].max()*100:.1f} cm")
    print(f"  Z: {reachable[:,2].min()*100:.1f} to {reachable[:,2].max()*100:.1f} cm")
    print("=== Done ===\n")
    data.qpos[:6] = original_qpos
    mujoco.mj_kinematics(model, data)
    viewer.sync()


def reset_trail(viewer, plane_center: np.ndarray, half: float) -> None:
    """
    Clear all user geoms and redraw just the plane outline.

    Also resets the global trail_full_warned flag so the next overflow
    on a fresh drawing prints its warning again.

    Args:
        viewer: passive viewer (uses viewer.user_scn)
        plane_center: 3D plane centre
        half: plane half-size in metres
    """
    global trail_full_warned
    viewer.user_scn.ngeom = 0
    draw_plane_outline(viewer.user_scn, plane_center, half)
    trail_full_warned = False


def run_drawing(model, data, viewer, led_site_id: int,
                plane_center: np.ndarray, half: float, path: Path,
                arm_ser=None) -> None:
    """
    Reset the arm to home, clear the trail, and execute the full path.

    Args:
        model: MjModel
        data: MjData
        viewer: passive viewer
        led_site_id: id of the led_tip site
        plane_center: 3D plane centre (used by reset_trail)
        half: plane half-size (used by reset_trail)
        path: list of waypoints to draw
        arm_ser: optional open serial connection to the real arm
    """
    data.qpos[:6] = HOME_QPOS
    data.ctrl[:6] = HOME_QPOS[:]
    mujoco.mj_kinematics(model, data)
    reset_trail(viewer, plane_center, half)
    viewer.sync()

    print("\n=== Drawing start ===\n")
    execute_path(model, data, viewer, led_site_id, path, arm_ser=arm_ser)
    print("\n=== Drawing complete. SPACE to replay. ===\n")


# ============================================================================
# SETUP
# ============================================================================

def setup_model() -> tuple:
    """
    Load scene XML, set the home pose, and return the bound MuJoCo objects.

    mj_kinematics propagates qpos → world-frame site/body poses without
    running full dynamics — exactly what we need for position-only IK.

    Returns:
        (model, data, led_site_id) tuple.
    """
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    data.qpos[:6] = HOME_QPOS
    mujoco.mj_kinematics(model, data)

    led_site_id = model.site("led_tip").id
    return model, data, led_site_id


def compute_plane(led_pos: np.ndarray, half: float) -> np.ndarray:
    """
    Compute the 3D centre of the drawing plane from the LED tip position.

    The plane is axis-aligned in XZ, PLANE_DISTANCE in front of the LED
    (along -Y), with its top edge at LED height — so the centre sits at
    led_pos.z - half.

    Args:
        led_pos: current world-frame LED tip position
        half: plane half-size in metres
    Returns:
        3D plane centre as np.ndarray of shape (3,).
    """
    return np.array([
        led_pos[0],
        led_pos[1] - PLANE_DISTANCE,
        led_pos[2] - half,
    ])


# ============================================================================
# MAIN
# ============================================================================

def main():
    """
    Orchestrate setup, build the path, and launch the viewer loop.

    Calls setup_model() and compute_plane(), maps strokes to a 3D path,
    wires up the keyboard callback, and dispatches SPACE/0 events from the
    main loop while the passive viewer is open.
    """
    if len(sys.argv) > 1:
        strokes = load_drawing(sys.argv[1])
    else:
        strokes = test_drawing()

    model, data, led_site_id = setup_model()
    led_pos = data.site_xpos[led_site_id].copy()

    arm_ser = None
    if RUN_REAL_ARM:
        arm_ser = serial.Serial(ARM_PORT, ARM_BAUD, timeout=2)
        import time as _t; _t.sleep(2)
        print("Real arm connected")

    # Apply the scale factor so both the plane outline and stroke mapping grow.
    half = HALF * DRAW_SCALE
    plane_center = compute_plane(led_pos, half)

    # 2D strokes → 3D target list (one dict per waypoint with pen_down flag).
    path = strokes_to_path(strokes, plane_center, half)
    print(f"\nPlane centre: [{plane_center[0]*100:.1f}, {plane_center[1]*100:.1f}, {plane_center[2]*100:.1f}] cm")
    print(f"Plane size: {half*200:.0f} × {half*200:.0f} cm  (DRAW_SCALE={DRAW_SCALE})")
    print(f"Path: {len(path)} points, {len(strokes)} strokes\n")

    # The passive viewer fires key_callback on each keypress. We just set
    # flags and react from the main loop so we don't do heavy work on the UI thread.
    state = AppState()

    def key_callback(keycode):
        if keycode == GLFW_KEY_SPACE:
            state.run_requested = True
        if keycode == GLFW_KEY_0:
            state.sweep_requested = True

    try:
        with mujoco.viewer.launch_passive(model, data,
                                          key_callback=key_callback) as viewer:
            mujoco.mj_kinematics(model, data)
            draw_plane_outline(viewer.user_scn, plane_center, half)
            viewer.sync()
            print("=== Ready. SPACE = draw, 0 = sweep ===\n")

            while viewer.is_running():
                if state.run_requested:
                    state.run_requested = False
                    run_drawing(model, data, viewer, led_site_id,
                                plane_center, half, path, arm_ser=arm_ser)
                elif state.sweep_requested:
                    state.sweep_requested = False
                    sweep_workspace(model, data, viewer, led_site_id)
                else:
                    mujoco.mj_kinematics(model, data)
                    viewer.sync()
    finally:
        if arm_ser is not None:
            arm_ser.close()


if __name__ == "__main__":
    main()
