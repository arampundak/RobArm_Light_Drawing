# arm_executor.py
#
# Bridge between MuJoCo joint angles (radians) and real-arm serial commands.
#
# Sends one MOVE,motor_id,position,speed\n line per joint over the XIAO
# serial link, with a 20 ms inter-command gap that the firmware needs.
# We send the *target* position once and let the SCServo's internal
# trajectory generator handle smoothing — no interpolated steps.
#
# Designed to be called once per IK waypoint (or once per IK step) from
# trace_plane.py once the arm is wired up.

import time

import serial

from arm_config import CALIBRATION


# ============================================================================
# NAMED POSES (raw encoder counts, keyed by arm_config name)
# ============================================================================
#
# Counts here go straight to the servos via send_joint_counts — no radian
# conversion. Useful for hardware-defined poses like the mechanical rest.

POSES = {
    "rest_pose": {
        "shoulder_pan":  3270,
        "shoulder_lift": 1760,
        "elbow_flex":    3289,
        "wrist_flex":    3858,
        "wrist_roll":    3000,
        "gripper":        906,
    },
}


# ============================================================================
# JOINT MAPPING
# ============================================================================
#
# MuJoCo's qpos[:6] order, the arm_config calibration name, and the servo
# motor ID on the daisy-chained bus.

MUJOCO_TO_ARM = [
    ("Rotation",    "shoulder_pan",  1),
    ("Pitch",       "shoulder_lift", 2),
    ("Elbow",       "elbow_flex",    3),
    ("Wrist_Pitch", "wrist_flex",    4),
    ("Wrist_Roll",  "wrist_roll",    5),
    ("Jaw",         "gripper",       6),
]

# MuJoCo joint ranges in radians, taken from so_arm100.xml joint limits.
MUJOCO_RANGES = {
    "Rotation":    (-1.92,  1.92),
    "Pitch":       (-3.32,  0.174),
    "Elbow":       (-0.174, 3.14),
    "Wrist_Pitch": (-1.66,  1.66),
    "Wrist_Roll":  (-2.79,  2.79),
    "Jaw":         (-0.174, 1.75),
}

# Quick lookup: MuJoCo name → arm_config name.
_MJ_TO_ARM_NAME = {mj: arm for (mj, arm, _) in MUJOCO_TO_ARM}

# Per-joint offsets for aligning MuJoCo's radian frame to the physical arm.
# The shoulder pan's drawing-home qpos should be the real arm's middle count.
MUJOCO_CENTER_QPOS = {
    "Rotation": -0.0576,
}


# ============================================================================
# SPEED
# ============================================================================
#
# SCServo speed units are effectively reversed on these servos:
#   0 = fastest / harshest, 4000 = slowest.
# Testing showed 2500 is the smooth working value for this arm.
# We use two regimes:
#   • DRAW — slow, for pen-down strokes (horizontal / upward moves)
#   • MOVE — faster, for pen-up repositioning. Also forced when the
#     shoulder_lift is descending, since gravity makes a slow descent
#     oscillate / stall.

SPEED_DRAW = 2500
SPEED_MOVE = 2500

# Testing showed st.WritePosEx(..., 2500, 500) is smooth on this arm.
ACC = 500

INTER_CMD_DELAY = 0.02   # 20 ms between MOVE lines — firmware needs this
COMMAND_DEADBAND_COUNTS = 8

BAUD       = 115200
DEFAULT_PORT = "/dev/cu.usbmodem1101"


# ============================================================================
# CONVERSION
# ============================================================================

def radians_to_counts(mujoco_name: str, angle_rad: float) -> int:
    """
    Linearly map a MuJoCo radian angle to a raw servo encoder count.

    Uses MUJOCO_RANGES[mujoco_name] for the input range and
    CALIBRATION[arm_name]["min"/"max"] for the output range. Clamps the
    input to the MuJoCo joint range so out-of-range angles map to the
    nearest valid encoder count.

    Args:
        mujoco_name: joint name as it appears in MUJOCO_TO_ARM (e.g. "Pitch")
        angle_rad: joint angle in radians (qpos value from MuJoCo)
    Returns:
        Integer encoder count to send to the servo.
    """
    rad_min, rad_max = MUJOCO_RANGES[mujoco_name]
    angle_clamped = max(rad_min, min(rad_max, angle_rad))

    arm_name = _MJ_TO_ARM_NAME[mujoco_name]
    cal = CALIBRATION[arm_name]
    cnt_min, cnt_max = cal["min"], cal["max"]

    t = (angle_clamped - rad_min) / (rad_max - rad_min)
    count = cnt_min + t * (cnt_max - cnt_min)
    if mujoco_name in MUJOCO_CENTER_QPOS:
        center_qpos = MUJOCO_CENTER_QPOS[mujoco_name]
        center_qpos = max(rad_min, min(rad_max, center_qpos))
        center_t = (center_qpos - rad_min) / (rad_max - rad_min)
        center_count = cnt_min + center_t * (cnt_max - cnt_min)
        count += cal["neutral"] - center_count
    count = max(cnt_min, min(cnt_max, count))
    return int(round(count))


# ============================================================================
# SPEED SELECTION
# ============================================================================

def pick_speed(current_counts: dict, target_counts: dict, pen_down: bool) -> int:
    """
    Choose a single SCServo speed for this batch of MOVE commands.

    Rules:
      • pen_down  → SPEED_DRAW (slow, smooth strokes)
      • pen_up    → SPEED_MOVE (fast repositioning)
      • Exception: if the shoulder_lift target is below the current count
        (i.e. the arm is descending), always use SPEED_MOVE regardless
        of pen state — slow descents fight gravity and oscillate.

    Args:
        current_counts: dict {arm_name: count} of where the arm is now
        target_counts:  dict {arm_name: count} of where it's going
        pen_down: whether this move is a drawing stroke
    Returns:
        SCServo speed value (0..4000).
    """
    cur_lift = current_counts.get("shoulder_lift")
    tgt_lift = target_counts.get("shoulder_lift")
    if cur_lift is not None and tgt_lift is not None and tgt_lift < cur_lift:
        return SPEED_MOVE

    return SPEED_DRAW if pen_down else SPEED_MOVE


# ============================================================================
# SEND
# ============================================================================

def send_joint_counts(
    ser: serial.Serial,
    target_counts: dict,
    pen_down: bool,
    current_counts: dict | None = None,
) -> dict:
    """
    Send a {arm_name: encoder_count} dict to the arm — no radian conversion.

    Sends one MOVE,motor_id,count,speed\\n line per joint with a 20 ms gap
    between commands (firmware requires this — do not remove). Sends the
    *target* count once per joint; the SCServo's trajectory generator
    smooths the motion, so no Python-side interpolation is performed.

    Use this when you already have hardware-defined counts (e.g. POSES,
    calibration neutrals). Use send_joint_angles when starting from
    MuJoCo radians.

    Args:
        ser: open pyserial connection to the XIAO (115200 baud, lf-terminated)
        target_counts: dict {arm_name: count} for all 6 joints
        pen_down: True = drawing stroke (slow); False = repositioning (fast)
        current_counts: previous joint counts for direction detection.
                        If None, uses SPEED_DRAW for all joints.
    Returns:
        dict {arm_name: count} of the counts just sent. Pass this back as
        current_counts on the next call so pick_speed can detect descents.
    """
    # Pick a single speed for this batch.
    if current_counts is None:
        speed = SPEED_DRAW
    else:
        speed = pick_speed(current_counts, target_counts, pen_down)

    sent_counts = {} if current_counts is None else current_counts.copy()

    # Fire the MOVE lines, one per joint, with the mandatory inter-cmd gap.
    for _mj_name, arm_name, motor_id in MUJOCO_TO_ARM:
        count = target_counts[arm_name]
        current = None if current_counts is None else current_counts.get(arm_name)
        if current is not None and abs(count - current) <= COMMAND_DEADBAND_COUNTS:
            continue

        cmd = f"MOVE,{motor_id},{count},{speed},{ACC}\n"
        ser.write(cmd.encode())
        time.sleep(INTER_CMD_DELAY)
        sent_counts[arm_name] = count

    return sent_counts


def send_joint_angles(
    ser: serial.Serial,
    qpos_radians: list[float],
    pen_down: bool,
    current_counts: dict | None = None,
) -> dict:
    """
    Convert 6 MuJoCo joint angles to encoder counts and send them to the arm.

    Thin wrapper: converts every angle via radians_to_counts, then defers
    to send_joint_counts for transmission and speed selection.

    Args:
        ser: open pyserial connection to the XIAO (115200 baud, lf-terminated)
        qpos_radians: 6 floats in MuJoCo order
                      [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw]
        pen_down: True = drawing stroke (slow); False = repositioning (fast)
        current_counts: previous joint counts for direction detection.
                        If None, uses SPEED_DRAW for all joints.
    Returns:
        dict {arm_name: count} of the counts just sent.
    """
    target_counts: dict = {}
    for (mj_name, arm_name, _motor_id), angle in zip(MUJOCO_TO_ARM, qpos_radians):
        target_counts[arm_name] = radians_to_counts(mj_name, angle)
    return send_joint_counts(ser, target_counts, pen_down, current_counts)


# ============================================================================
# TEST
# ============================================================================

if __name__ == "__main__":
    import serial
    import time

    HOME_QPOS = [-0.0576, -2.03, 0.837, 1.08, 0.0837, 0.0]

    with serial.Serial("/dev/cu.usbmodem1101", 115200, timeout=2) as ser:
        time.sleep(2)

        print("Sending home pose to real arm...")
        counts = send_joint_angles(ser, HOME_QPOS, pen_down=False)
        print("Done — arm should be in drawing home position")

        try:
            input("\nPress Enter to return to rest pose (Ctrl-C to abort)... ")
        except (KeyboardInterrupt, EOFError):
            print("\nAborted — arm left in home pose.")
        else:
            print("Sending rest pose...")
            send_joint_counts(ser, POSES["rest_pose"], pen_down=False, current_counts=counts)
            print("Done — arm at rest.")
