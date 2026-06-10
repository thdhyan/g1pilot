#!/usr/bin/env python3
"""Standalone G1 motor-state reader (run on the robot's onboard Jetson).

Subscribes to the Unitree DDS topic `rt/lowstate` and prints per-joint
position / velocity / estimated torque / temperature, plus a header line with
the overarching state (IMU, update rate, dropped packets).

Only dependency is unitree_sdk2py. The g1pilot package is used if importable
(for real joint names) but is NOT required.

Usage:
    python3 read_motor_state.py [INTERFACE] [--hz RATE] [--once] [--raw]

    INTERFACE   network interface facing the robot (default: $G1_INTERFACE).
                On the G1 onboard computer this is the internal wired link.
    --hz RATE   print rate in Hz (default 5). The DDS sample rate is unchanged;
                this only throttles printing.
    --once      print a single snapshot and exit (good for scripting/asserts).
    --raw       print as a flat list of q values (easy to copy into q_init).
"""
import argparse
import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

G1_NUM_MOTOR = 29

# Real joint names if the package is on the path; otherwise an inlined fallback
# (index order matches G1_29_JointIndex).
try:
    from g1pilot.utils.common import G1_29_JointIndex
    JOINT_NAMES = [G1_29_JointIndex(i).name[1:] for i in range(G1_NUM_MOTOR)]
except Exception:
    JOINT_NAMES = [
        "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
        "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
        "WaistYaw", "WaistRoll", "WaistPitch",
        "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow",
        "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
        "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow",
        "RightWristRoll", "RightWristPitch", "RightWristYaw",
    ]


# Unitree wireless-remote button bitmask (bytes [2:4], little-endian).
REMOTE_KEYS = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
               "A", "B", "X", "Y", "up", "right", "down", "left"]


def _field_names(obj):
    """Best-effort field list for a cyclonedds IDL dataclass (no SDK import)."""
    fields = getattr(obj, "__dataclass_fields__", None)
    if fields:
        return list(fields.keys())
    return [a for a in dir(obj) if not a.startswith("_") and not callable(getattr(obj, a, None))]


def _dump_obj(obj, indent="    "):
    """Print scalar fields of a nested IDL object (imu_state, bms_state, ...)."""
    out = []
    for name in _field_names(obj):
        try:
            val = getattr(obj, name)
        except Exception:
            continue
        if callable(val):
            continue
        # Shorten long arrays so the dump stays readable.
        if hasattr(val, "__len__") and not isinstance(val, (str, bytes)) and len(val) > 8:
            val = f"[{len(val)} items] " + ", ".join(str(v) for v in list(val)[:8]) + " ..."
        out.append(f"{indent}{name:<16} {val}")
    return "\n".join(out)


def _decode_remote(raw):
    """Decode the 40-byte wireless_remote blob into pressed-button names."""
    try:
        b = bytes(raw)
    except Exception:
        return f"(uninterpretable: {raw!r})"
    if len(b) < 4:
        return f"(too short: {b!r})"
    mask = b[2] | (b[3] << 8)
    pressed = [REMOTE_KEYS[i] for i in range(16) if mask & (1 << i)]
    return f"btn_mask=0x{mask:04x} pressed={pressed or '[]'}  raw[0:8]={b[:8].hex()}"


def fmt_fault(msg):
    """Passive, one-shot dump of the robot's self-reported fault state."""
    lines = ["===== G1 FAULT DUMP ====="]
    lines.append(f"tick={getattr(msg,'tick','?')}  mode_machine={getattr(msg,'mode_machine','?')}  "
                 f"mode_pr={getattr(msg,'mode_pr','?')}  crc={getattr(msg,'crc','?')}")

    imu = getattr(msg, "imu_state", None)
    if imu is not None:
        lines.append("IMU:")
        lines.append(_dump_obj(imu))

    # Battery / power: name varies across IDLs (bms_state / power_v / power_a).
    for pname in ("bms_state", "power_v", "power_a"):
        if hasattr(msg, pname):
            val = getattr(msg, pname)
            if hasattr(val, "__dataclass_fields__") or _field_names(val):
                lines.append(f"{pname}:")
                lines.append(_dump_obj(val))
            else:
                lines.append(f"{pname:<10} {val}")

    wr = getattr(msg, "wireless_remote", None)
    if wr is not None:
        lines.append("wireless_remote: " + _decode_remote(wr))

    # Per-motor: flag anything not actively controlled (mode 0) or faulted.
    lines.append(f"{'idx':>3} {'joint':<20} {'mode':>4} {'err':>5} "
                 f"{'T[C]':>6} {'vol[V]':>7} {'tau[Nm]':>9}  flags")
    n_mode0 = n_err = 0
    for i in range(G1_NUM_MOTOR):
        ms = msg.motor_state[i]
        mode = getattr(ms, "mode", "-")
        err = getattr(ms, "motorstate", getattr(ms, "reserve", "-"))
        temp = getattr(ms, "temperature", "-")
        try:
            temp = max(temp)
        except TypeError:
            pass
        vol = getattr(ms, "vol", "-")
        tau = getattr(ms, "tau_est", float("nan"))
        flags = []
        if mode == 0:
            flags.append("MODE0"); n_mode0 += 1
        if isinstance(err, int) and err != 0:
            flags.append(f"ERR={err}"); n_err += 1
        vol_s = f"{vol:7.2f}" if isinstance(vol, (int, float)) else f"{str(vol):>7}"
        lines.append(
            f"{i:>3} {JOINT_NAMES[i]:<20} {str(mode):>4} {str(err):>5} "
            f"{str(temp):>6} {vol_s} {tau:>9.3f}  {' '.join(flags)}"
        )
    lines.append(f"--- summary: {n_mode0}/{G1_NUM_MOTOR} motors at mode=0, "
                 f"{n_err} reporting a motor error ---")
    return "\n".join(lines)


def fmt_snapshot(msg, raw=False):
    if raw:
        return "[" + ", ".join(f"{msg.motor_state[i].q:.4f}" for i in range(G1_NUM_MOTOR)) + "]"

    lines = []
    # tick is a rolling counter; crc/mode_machine help confirm a live link.
    lines.append(
        f"tick={getattr(msg, 'tick', '?')}  mode_machine={getattr(msg, 'mode_machine', '?')}"
    )
    lines.append(f"{'idx':>3} {'joint':<20} {'q[rad]':>9} {'dq[rad/s]':>10} "
                 f"{'tau[Nm]':>9} {'T[C]':>6} {'mode':>5} {'lost':>5}")
    for i in range(G1_NUM_MOTOR):
        ms = msg.motor_state[i]
        # temperature is a 2-element array on G1 [chip, motor]; show the hotter.
        temp = getattr(ms, "temperature", "-")
        try:
            temp = max(temp)
        except TypeError:
            pass
        # tau_est / mode / lost aren't present on every IDL variant
        # (e.g. unitree_hg MotorState_ has no `lost`); degrade gracefully.
        tau = getattr(ms, "tau_est", float("nan"))
        mode = getattr(ms, "mode", "-")
        lost = getattr(ms, "lost", "-")
        lines.append(
            f"{i:>3} {JOINT_NAMES[i]:<20} {ms.q:>9.4f} {ms.dq:>10.4f} "
            f"{tau:>9.3f} {str(temp):>6} {str(mode):>5} {str(lost):>5}"
        )
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description="Read G1 motor state over DDS.")
    p.add_argument("interface", nargs="?", default=os.environ.get("G1_INTERFACE"),
                   help="network interface facing the robot (default: $G1_INTERFACE)")
    p.add_argument("--hz", type=float, default=5.0, help="print rate (default 5)")
    p.add_argument("--once", action="store_true", help="print one snapshot and exit")
    p.add_argument("--raw", action="store_true", help="print flat q list only")
    p.add_argument("--fault", action="store_true",
                   help="one-shot fault dump (IMU, battery, remote, per-motor mode/err) and exit")
    args = p.parse_args()

    if not args.interface:
        sys.exit("ERROR: no interface. Pass one or set G1_INTERFACE "
                 "(e.g. the onboard wired link to the motor board).")

    ChannelFactoryInitialize(0, args.interface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()

    # Wait for the first sample so we don't print None on a dead link.
    print(f"Subscribing to rt/lowstate on '{args.interface}' ...", file=sys.stderr)
    deadline = None
    first = None
    while first is None:
        first = sub.Read()
        if first is None:
            time.sleep(0.01)
            deadline = (deadline or 0) + 1
            if deadline > 500:  # ~5 s
                sys.exit("ERROR: no LowState received in 5 s. Wrong interface, "
                         "robot off, or DDS domain mismatch?")

    if args.fault:
        print(fmt_fault(first), flush=True)
        return

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    last = first
    while True:
        msg = sub.Read()
        if msg is not None:
            last = msg
        print("\n" + fmt_snapshot(last, raw=args.raw), flush=True)
        if args.once:
            break
        time.sleep(period)


if __name__ == "__main__":
    main()
