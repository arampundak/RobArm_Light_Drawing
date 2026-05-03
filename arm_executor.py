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
# READ
# ============================================================================

def enable_torque(ser: serial.Serial, motor_id: int, enable: bool = True) -> bool:
    """
    Send TORQUE,id,enable to the firmware and wait for the OK ack.

    SCServo torque state is sticky across XIAO reboots — it only resets when
    the servo itself loses 12 V. If a previous session disabled torque on a
    joint (e.g. to free-spin it for calibration), WritePosEx will silently
    no-op until torque is re-enabled. Call this on every motor at startup
    to make sure the arm is actually holding.

    Args:
        ser: open pyserial connection to the XIAO
        motor_id: SCServo bus ID (1..6)
        enable: True = hold position, False = release (free-spin)
    Returns:
        True if the firmware acknowledged within 1 s, False on timeout.
    """
    ser.reset_input_buffer()
    ser.write(f"TORQUE,{motor_id},{1 if enable else 0}\n".encode())
    old_timeout = ser.timeout
    ser.timeout = 1.0
    try:
        for _ in range(8):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            if line.startswith(f"OK,TORQUE,{motor_id},"):
                return True
    finally:
        ser.timeout = old_timeout
    return False


def read_joint(ser: serial.Serial, motor_id: int, timeout_s: float = 1.0) -> dict | None:
    """
    Query a single servo for position, load, voltage, and temperature.

    Sends READ,id\\n and parses the firmware's
    OK,READ,id,pos,load,volt_dV,temp_C line. Drains stale bytes first so
    leftover MOVE acks don't get confused with the read response.

    Args:
        ser: open pyserial connection to the XIAO
        motor_id: SCServo bus ID (1..6)
        timeout_s: max seconds to wait for the response line
    Returns:
        dict {"pos", "load", "volt", "temp"} as ints, or None if the servo
        did not answer (firmware returns -1 from ReadPos on bus failure).
    """
    ser.reset_input_buffer()
    ser.write(f"READ,{motor_id}\n".encode())

    old_timeout = ser.timeout
    ser.timeout = timeout_s
    try:
        # The firmware may emit other lines first (stale OK from MOVEs,
        # READY on boot). Skim a few lines looking for our specific match.
        for _ in range(8):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            if not line.startswith(f"OK,READ,{motor_id},"):
                continue
            parts = line.split(",")
            if len(parts) != 7:
                continue
            try:
                pos  = int(parts[3])
                load = int(parts[4])
                volt = int(parts[5])
                temp = int(parts[6])
            except ValueError:
                continue
            if pos == -1:
                return None
            return {"pos": pos, "load": load, "volt": volt, "temp": temp}
    finally:
        ser.timeout = old_timeout
    return None


def read_limits(ser: serial.Serial, motor_id: int, timeout_s: float = 1.0) -> tuple[int, int] | None:
    """
    Read the EEPROM angle-limit registers from a single SMS_STS servo.

    Sends LIMITS,id\\n and parses OK,LIMITS,id,min_angle,max_angle. The
    servo silently clamps any WritePosEx target to [min_angle, max_angle],
    so this is what to inspect when a motor stops short of its commanded
    target with no load and no temperature trip.

    Args:
        ser: open pyserial connection to the XIAO
        motor_id: SCServo bus ID (1..6)
        timeout_s: max seconds to wait for the response line
    Returns:
        (min_angle, max_angle) tuple, or None if the servo did not answer
        (firmware returns -1,-1 on bus failure).
    """
    ser.reset_input_buffer()
    ser.write(f"LIMITS,{motor_id}\n".encode())

    old_timeout = ser.timeout
    ser.timeout = timeout_s
    try:
        for _ in range(8):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            if not line.startswith(f"OK,LIMITS,{motor_id},"):
                continue
            parts = line.split(",")
            if len(parts) != 5:
                continue
            try:
                lo = int(parts[3])
                hi = int(parts[4])
            except ValueError:
                continue
            if lo == -1 or hi == -1:
                return None
            return (lo, hi)
    finally:
        ser.timeout = old_timeout
    return None


# ============================================================================
# TEST / DIAGNOSTIC
# ============================================================================

if __name__ == "__main__":
    import sys
    import time

    HOME_QPOS = [-0.0576, -2.03, 0.837, 1.08, 0.0837, 0.0]

    def _read_all(ser):
        """Return {arm_name: read_joint_result_or_None} for all 6 motors."""
        out = {}
        for _, arm_name, motor_id in MUJOCO_TO_ARM:
            out[arm_name] = read_joint(ser, motor_id)
        return out

    def _print_bus(snapshot):
        print(f" {'id':>2} {'name':>14}   {'pos':>5}  {'load':>6}  {'V_dV':>4}  {'T_C':>3}")
        for _, arm_name, motor_id in MUJOCO_TO_ARM:
            r = snapshot[arm_name]
            if r is None:
                print(f" {motor_id:>2} {arm_name:>14}   NO RESPONSE")
            else:
                print(f" {motor_id:>2} {arm_name:>14}   {r['pos']:>5}  {r['load']:>6}  {r['volt']:>4}  {r['temp']:>3}")

    print(f"Opening serial to {DEFAULT_PORT} @ {BAUD}")
    with serial.Serial(DEFAULT_PORT, BAUD, timeout=1) as ser:
        time.sleep(2)               # XIAO boot
        ser.reset_input_buffer()    # drop the "READY" greeting

        # ----- Phase 1: bus check -----------------------------------------
        # Just read every motor. If any are missing here, no MOVE will help.
        print("\n=== Phase 1: bus check (READ each motor) ===")
        baseline = _read_all(ser)
        _print_bus(baseline)

        missing = [n for _, n, _ in MUJOCO_TO_ARM if baseline[n] is None]
        if missing:
            print(f"\nNo response from: {', '.join(missing)}")
            print("Likely daisy-chain break, ID conflict, or that servo lost power.")
            print("Stopping before any MOVE is sent.")
            sys.exit(1)
        print("All 6 motors answered.")

        # Force torque ON for every motor. SCServo torque state is sticky:
        # if anything ever sent TORQUE,id,0 to release a joint, WritePosEx
        # would silently no-op on it until re-enabled here.
        print("\nEnabling torque on all 6 motors...")
        for _, arm_name, motor_id in MUJOCO_TO_ARM:
            ok = enable_torque(ser, motor_id, True)
            print(f"  {motor_id} {arm_name:>14}  {'OK' if ok else 'NO ACK'}")

        # Read EEPROM angle limits and compare to what arm_config expects.
        # If a servo's limit is narrower than the cal range, MOVE commands
        # past the limit are silently clamped — looks like "motor stops short."
        print("\nReading EEPROM angle limits...")
        print(f" {'id':>2} {'name':>14}   {'srv_min':>7}  {'srv_max':>7}    {'cfg_min':>7}  {'cfg_max':>7}    {'verdict':>20}")
        for _, arm_name, motor_id in MUJOCO_TO_ARM:
            limits = read_limits(ser, motor_id)
            cfg = CALIBRATION[arm_name]
            cfg_lo, cfg_hi = cfg["min"], cfg["max"]
            if limits is None:
                print(f" {motor_id:>2} {arm_name:>14}   {'?':>7}  {'?':>7}    {cfg_lo:>7}  {cfg_hi:>7}    {'NO RESPONSE':>20}")
                continue
            srv_lo, srv_hi = limits
            verdict = "ok"
            if srv_lo > cfg_lo + 5:
                verdict = f"min too high (+{srv_lo - cfg_lo})"
            elif srv_hi < cfg_hi - 5:
                verdict = f"max too low ({srv_hi - cfg_hi})"
            print(f" {motor_id:>2} {arm_name:>14}   {srv_lo:>7}  {srv_hi:>7}    {cfg_lo:>7}  {cfg_hi:>7}    {verdict:>20}")

        # ----- Phase 2: batch HOME_QPOS — reproduces the failing path ----
        # Identical to what trace_plane and the old test did: send all six
        # MOVEs as one batch, then look at who actually moved.
        try:
            input("\n[Phase 2] Press Enter to send HOME_QPOS as a batch (Ctrl-C to abort)... ")
        except (KeyboardInterrupt, EOFError):
            print("\nAborted before Phase 2.")
            sys.exit(0)

        targets = {}
        for (mj_name, arm_name, _id), angle in zip(MUJOCO_TO_ARM, HOME_QPOS):
            targets[arm_name] = radians_to_counts(mj_name, angle)

        before = _read_all(ser)
        send_joint_counts(ser, targets, pen_down=False)
        print("\nSent batch — polling positions live for 15 s.")
        print("Watching for: motors that move and stop (good), motors that don't move at all,")
        print("and motors that move then drift backwards (gravity winning = no torque).\n")

        # Header: t_s | per-motor commanded values, then live position stream.
        names = [n for _, n, _ in MUJOCO_TO_ARM]
        print(" cmd  " + "  ".join(f"{targets[n]:>5}" for n in names))
        print(" t_s  " + "  ".join(f"{n[:5]:>5}" for n in names))
        print("-" * (6 + 7 * len(names)))

        start = time.time()
        after = before
        while True:
            elapsed = time.time() - start
            if elapsed > 15:
                break
            snap = _read_all(ser)
            after = snap
            cells = []
            for n in names:
                r = snap[n]
                cells.append(" ----" if r is None else f"{r['pos']:>5}")
            print(f"{elapsed:>4.1f}  " + "  ".join(cells))
            time.sleep(0.5)

        print("\n=== Phase 2 result: HOME batch MOVE ===")
        print(f" {'id':>2} {'name':>14}  {'cmd':>5}  {'before':>6} {'after':>6}  {'delta':>6}  {'load':>6}  {'V_dV':>4}  {'T_C':>3}")
        for _, arm_name, motor_id in MUJOCO_TO_ARM:
            cmd = targets[arm_name]
            b = before[arm_name]
            a = after[arm_name]
            b_str = "----" if b is None else f"{b['pos']:>6}"
            a_str = "----" if a is None else f"{a['pos']:>6}"
            if b is not None and a is not None:
                d_str = f"{a['pos'] - b['pos']:+6d}"
            else:
                d_str = "   ?  "
            l_str = "----" if a is None else f"{a['load']:>6}"
            v_str = "----" if a is None else f"{a['volt']:>4}"
            t_str = "----" if a is None else f"{a['temp']:>3}"
            print(f" {motor_id:>2} {arm_name:>14}  {cmd:>5}  {b_str} {a_str}  {d_str}  {l_str}  {v_str}  {t_str}")

        print("\nWhat to look for:")
        print("  - delta ~ 0 with after far from cmd  -> command ignored or torque off")
        print("  - delta ~ cmd-before                 -> motor obeyed the command")
        print("  - large |load|                       -> servo straining (mech bind / overload)")
        print("  - V_dV (deci-volts) far below ~120   -> brownout under load")
        print("  - T_C high (>60)                     -> thermal protection may engage")
        print("\n[Phase 3] Press Enter to send rest_pose (Ctrl-C to skip)... ", end="", flush=True)
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            print("\nDone.")
            sys.exit(0)

        send_joint_counts(ser, POSES["rest_pose"], pen_down=False)
        time.sleep(3)
        rest_after = _read_all(ser)
        print("\n=== Phase 3 result: rest_pose ===")
        _print_bus(rest_after)
        print("Done.")
