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

import mujoco
import mujoco.viewer
import numpy as np

# ============================================================================
# CONFIGURATION
# ============================================================================

SCENE_XML = os.path.join(os.path.dirname(__file__), "scene.xml") #— loads so_arm100.xml and defines the drawing plane

# Starting joint angles (radians) — arm pre-posed near the drawing plane so
# the IK loop doesn't have to travel far to reach the first stroke.
HOME_QPOS = [-0.0576, -2.03, 0.837, 1.08, 0.0837, 0.0] #— arm pre-posed near the plane, not straight out, to help IK converge to the first stroke, which is near the plane's top edge. These were found by manual tuning: loading the model in the viewer, dragging the arm into a good starting pose, and reading off the qpos from data.qpos.

# IK tuning — damped least squares (see move_to_target for the math).
IK_DT        = 0.05     # integration step applied to dq each iteration, smaller = more stable but slower convergence, especially on long moves, and more likely to get stuck on IK_THRESHOLD if too small. This is not a real time step, just a numerical integration parameter for the IK loop.
IK_DAMPING   = 0.01     # λ — larger = more stable near singularities, slower
IK_THRESHOLD = 0.005    # stop when position error < 5 mm
IK_MAX_STEPS = 3000     # hard cap per target point

# Drawing plane half-size → 20 cm × 20 cm total.
HALF = 0.1

# Scale factor for the drawing — 1.0 = default, 1.5 = 50% bigger, etc.
# Multiplies HALF so the plane (and drawing) grow together.
DRAW_SCALE = 2.0

# Per-step viewer pacing. DRAW_SLEEP is intentionally slower so the trail
# has time to appear on screen; MOVE_SLEEP is for pen-up traversals.
DRAW_SLEEP = 0.004
MOVE_SLEEP = 0.001

# GLFW key codes used by the passive viewer's key_callback.
GLFW_KEY_SPACE = 32
GLFW_KEY_0     = 48

# ----- Trail / plane visualization -----
TRAIL_RADIUS  = 0.0008                 # capsule radius (~1.6 mm thick line)
TRAIL_MIN_SEG = 0.002                  # don't add segments shorter than 2 mm
TRAIL_RGBA    = (1.0, 0.15, 0.15, 1.0) # red pen line
PLANE_RGBA    = (0.2, 0.8, 1.0, 1.0)   # cyan plane outline
PLANE_RADIUS  = 0.0007


# ============================================================================
# LOAD DRAWING
# ============================================================================

def load_drawing(json_path):
    """Load strokes from a p5 export JSON file."""
    with open(json_path) as f:
        data = json.load(f)
    strokes: list[list[dict[str, float]]] = data["strokes"]
    '''A dict with "strokes" key, which is a list of strokes, where each stroke is a list of points, and each point is a dict with "x" and "y" keys.'''
    total_pts: int = sum(len(s) for s in strokes)
    print(f"Loaded {len(strokes)} strokes, {total_pts} points from {json_path}")
    return strokes


def test_drawing() -> list[list[dict[str, float]]]:
    """Built-in test: a triangle + a dot in the centre."""
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


def strokes_to_path(strokes, plane_center, half) -> list[dict[str, float]]:
    """
    Convert normalized 2D strokes to 3D path points on the drawing plane.

    Mapping:
        nx (0→1)  →  X: plane_center[0] - half  to  + half
        ny (0→1)  →  Z: plane_center[2] + half  to  - half  (flipped: screen-Y down → world-Z up)
        Y stays constant at plane_center[1]       (the depth of the plane)

    Between strokes: pen_down=False (arm lifts, moves to next stroke's start).
    Within a stroke : pen_down=True  (arm draws a line).
    """
    path = []
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

def add_capsule_segment(scn, p1, p2, radius, rgba):
    """Append a capsule between p1 and p2 to the user scene. Returns False if full."""
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


def draw_plane_outline(scn, center, half):
    """Four capsules framing the drawing plane (XZ rectangle at center[1])."""
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
# MAIN
# ============================================================================

def main():
    # ------------------------------------------------------------------
    # Load the drawing (from JSON argv or the built-in test triangle).
    # ------------------------------------------------------------------
    if len(sys.argv) > 1:
        strokes = load_drawing(sys.argv[1])
    else:
        strokes = test_drawing()

    # ------------------------------------------------------------------
    # Build the MuJoCo model and set the home configuration.
    # mj_kinematics propagates qpos → world-frame site/body poses without
    # running full dynamics — exactly what we need for position-only IK.
    # ------------------------------------------------------------------
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    data.qpos[:6] = HOME_QPOS
    mujoco.mj_kinematics(model, data)

    led_site_id = model.site("led_tip").id
    led_pos     = data.site_xpos[led_site_id].copy()

    # Apply the scale factor so both the plane outline and stroke mapping grow.
    half = HALF * DRAW_SCALE

    # Plane: axis-aligned in XZ, 5 cm in front of the LED, top edge at LED height.
    plane_center = np.array([
        led_pos[0],
        led_pos[1] - 0.05,
        led_pos[2] - half,
    ])

    # 2D strokes → 3D target list (one dict per waypoint with pen_down flag).
    path = strokes_to_path(strokes, plane_center, half)
    print(f"\nPlane centre: [{plane_center[0]*100:.1f}, {plane_center[1]*100:.1f}, {plane_center[2]*100:.1f}] cm")
    print(f"Plane size: {half*200:.0f} × {half*200:.0f} cm  (DRAW_SCALE={DRAW_SCALE})")
    print(f"Path: {len(path)} points, {len(strokes)} strokes\n")

    # Single-shot warning if the user_scn fills up (long drawings).
    trail_full_warned = {"flag": False}

    # ------------------------------------------------------------------
    # INVERSE KINEMATICS — damped least squares
    # ------------------------------------------------------------------
    # For a site position error e = target - current, the joint update is:
    #     dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
    # This handles near-singular configurations gracefully (unlike the raw
    # pseudoinverse) at the cost of slightly biased motion near singularities.
    #
    # Each iteration:
    #   1. Compute the 3×6 position Jacobian of the led_tip site.
    #   2. Solve the damped system for dq.
    #   3. Integrate qpos by dq * IK_DT, clip into joint limits.
    #   4. mj_kinematics + viewer.sync to redraw.
    #   5. If pen is down, append a red capsule to the trail.
    # ------------------------------------------------------------------
    def move_to_target(target, label, pen_down=True):
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
                        if not trail_full_warned["flag"]:
                            print("  [trail] user_scn full — further segments dropped")
                            trail_full_warned["flag"] = True
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
            # This keeps segment count manageable on long JSON drawings.
            if pen_down:
                new_pos = data.site_xpos[led_site_id].copy()
                if np.linalg.norm(new_pos - last_trail_pos) > TRAIL_MIN_SEG:
                    if not add_capsule_segment(viewer.user_scn, last_trail_pos,
                                               new_pos, TRAIL_RADIUS, TRAIL_RGBA):
                        if not trail_full_warned["flag"]:
                            print("  [trail] user_scn full — further segments dropped")
                            trail_full_warned["flag"] = True
                    last_trail_pos = new_pos

            viewer.sync()
            time.sleep(DRAW_SLEEP if pen_down else MOVE_SLEEP)

        print(f"  [{label}] max steps | err: {dist*100:.2f} cm")
        return False

    def execute_path(path_points):
        """Drive the arm through every target in order."""
        for i, pt in enumerate(path_points):
            target = np.array([pt["x"], pt["y"], pt["z"]])
            label  = f"pt{i+1}/{len(path_points)}"
            move_to_target(target, label, pen_down=pt["pen_down"])

    # ------------------------------------------------------------------
    # Workspace sweep — random joint samples to visualize the reachable
    # volume of the LED tip. Printed bounds only; no trail writes.
    # ------------------------------------------------------------------
    def sweep_workspace():
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

    # ------------------------------------------------------------------
    # Input handling — the passive viewer fires key_callback on each
    # keypress. We just set flags and react from the main loop so we
    # don't do heavy work on the UI thread.
    # ------------------------------------------------------------------
    state = {"run_requested": False, "sweep_requested": False}

    def key_callback(keycode):
        if keycode == GLFW_KEY_SPACE:
            state["run_requested"] = True
        if keycode == GLFW_KEY_0:
            state["sweep_requested"] = True

    def reset_trail():
        """Clear all user geoms and redraw just the plane outline."""
        viewer.user_scn.ngeom = 0
        draw_plane_outline(viewer.user_scn, plane_center, half)
        trail_full_warned["flag"] = False

    def run_drawing():
        data.qpos[:6] = HOME_QPOS
        data.ctrl[:6] = HOME_QPOS[:]
        mujoco.mj_kinematics(model, data)
        reset_trail()
        viewer.sync()

        print("\n=== Drawing start ===\n")
        execute_path(path)
        print("\n=== Drawing complete. SPACE to replay. ===\n")

    # ------------------------------------------------------------------
    # Launch the passive viewer and enter the interaction loop.
    # ------------------------------------------------------------------
    with mujoco.viewer.launch_passive(model, data,
                                       key_callback=key_callback) as viewer:
        mujoco.mj_kinematics(model, data)
        draw_plane_outline(viewer.user_scn, plane_center, half)
        viewer.sync()
        print("=== Ready. SPACE = draw, 0 = sweep ===\n")

        while viewer.is_running():
            if state["run_requested"]:
                state["run_requested"] = False
                run_drawing()
            elif state["sweep_requested"]:
                state["sweep_requested"] = False
                sweep_workspace()
            else:
                mujoco.mj_kinematics(model, data)
                viewer.sync()


if __name__ == "__main__":
    main()
