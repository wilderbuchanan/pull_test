#!/usr/bin/env python3
# m1_buttons_cli.py

import os
import sys
import time
from roboclaw_3 import Roboclaw

# Headless-safe plotting (no GUI needed)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- GPIO import with helpful error ----------
try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError:
    sys.stderr.write(
        "\nERROR: RPi.GPIO not found.\n"
        "Install it on Raspberry Pi OS:\n"
        "  sudo apt-get update && sudo apt-get install -y python3-rpi.gpio\n"
        "If you're in a virtualenv, also:\n"
        "  pip install RPi.GPIO\n\n"
    )
    sys.exit(1)

# ------------- User Settings -------------
PORT  = "/dev/ttyACM0"
BAUD  = 38400
ADDR  = 0x80

# Measured QPPS; set to None to auto-measure at startup
QPPS  = 4196

# Velocity PID (16.16 fixed-point in the call)
KP, KI, KD = 0.35, 0.007, 0.04

# Closed-loop “slower” target for button 1 (fraction of QPPS)
TARGET_FRACTION = 0.40

# Full-speed fraction for hold buttons
FULL_SPEED_FRACTION = 0.95  # ≈ 95% of QPPS

# Button pins (BCM numbering)
BTN_SEQ_BACK_THEN_RETURN = 17  # Button 1
BTN_HOLD_BACKWARD        = 27  # Button 2 (SWAPPED: now backward while held)
BTN_HOLD_FORWARD         = 22  # Button 3 (SWAPPED: now forward while held)

# Timing / debounce
SAMPLE_DT        = 0.10     # seconds between samples (for logging plot runs)
BACKWARD_SECONDS = 30       # Button 1: backward phase duration
DEBOUNCE_MS      = 30
RETURN_TOL       = 10       # counts: stop when within this of start encoder
RETURN_TIMEOUT   = 60       # safety limit (s) for return phase
# -----------------------------------------

rc = Roboclaw(PORT, BAUD)

# ------------ Helpers ------------
def fix16(x: float) -> int:
    return int(x * 65536.0)

def read_speed():
    """Return (ok, speed_cps)."""
    out = rc.ReadSpeedM1(ADDR)
    if isinstance(out, tuple) and len(out) >= 2:
        return bool(out[0]), int(out[1])
    return False, 0

def read_enc():
    """Return (ok, encoder_count)."""
    out = rc.ReadEncM1(ADDR)
    if isinstance(out, tuple) and len(out) >= 2:
        return bool(out[0]), int(out[1])
    return False, 0

def next_increment_path(dirpath: str, basename: str, ext: str = ".png") -> str:
    os.makedirs(dirpath, exist_ok=True)
    i = 1
    while True:
        candidate = os.path.join(dirpath, f"{basename}_{i}{ext}")
        if not os.path.exists(candidate):
            return candidate
        i += 1

def save_speed_plot(times, speeds, refs, out_dir="output_photos"):
    """Save a plot of measured speed vs reference; auto-increment filename."""
    if not times or not speeds or not refs:
        print("No data to plot.")
        return
    path = next_increment_path(out_dir, "speed_vs_reference", ".png")
    plt.figure(figsize=(10, 6))
    plt.plot(times, speeds, label="Measured Speed")
    plt.plot(times, refs, "--", label="Reference Speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (counts/sec)")
    plt.title("Motor Speed vs Reference")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved plot: {path}")

def measure_qpps(seconds=1.5):
    """Briefly run full forward & reverse and return the max cps observed."""
    samples = []

    def sample_for(sec, forward=True):
        if forward:
            rc.ForwardM1(ADDR, 127)
        else:
            rc.BackwardM1(ADDR, 127)
        t0 = time.time()
        while time.time() - t0 < sec:
            ok, spd = read_speed()
            if ok:
                samples.append(abs(spd))
            time.sleep(0.05)
        rc.ForwardM1(ADDR, 0)

    sample_for(seconds, True)
    time.sleep(0.2)
    sample_for(seconds, False)
    return max(samples) if samples else 0

def wait_for_press_stable(pin, timeout=None):
    """Wait until the pin is pressed (active-low) and stable for DEBOUNCE_MS."""
    t_start = time.time()
    while True:
        if GPIO.input(pin) == GPIO.LOW:
            time.sleep(DEBOUNCE_MS / 1000.0)
            if GPIO.input(pin) == GPIO.LOW:
                return True
        if timeout is not None and (time.time() - t_start) > timeout:
            return False
        time.sleep(0.005)

def wait_for_release_stable(pin):
    """Wait until the pin is released (goes HIGH) and is stable for DEBOUNCE_MS."""
    while True:
        if GPIO.input(pin) == GPIO.HIGH:
            time.sleep(DEBOUNCE_MS / 1000.0)
            if GPIO.input(pin) == GPIO.HIGH:
                return
        time.sleep(0.005)

# ------------ Button Actions ------------
def action_seq_back_then_return(target_slow_cps, target_full_fwd_cps):
    """
    Button 1 sequence:
      1) Record starting encoder
      2) Run backward for BACKWARD_SECONDS at target_slow_cps (log for plot)
      3) Run forward at target_full_fwd_cps until encoder returns to starting position (±RETURN_TOL)
      4) Save plot of the entire sequence
    """
    print("Button 1: Backward 30s (slow) -> Forward to start (full) + save plot")

    # Arrays for the whole sequence logging
    times, speeds, refs = [], [], []

    # Read starting encoder
    ok_e0, enc_start = read_enc()
    if not ok_e0:
        print("WARN: Could not read starting encoder; proceeding without return-to-start check.")
        enc_start = None

    # Phase 1: backward at slow speed for BACKWARD_SECONDS
    t0 = time.time()
    rc.SpeedM1(ADDR, -abs(target_slow_cps))
    try:
        while time.time() - t0 < BACKWARD_SECONDS:
            now = time.time() - t0
            ok_s, spd = read_speed()
            if ok_s:
                times.append(now)
                speeds.append(spd)
                refs.append(-abs(target_slow_cps))
            time.sleep(SAMPLE_DT)
    finally:
        rc.SpeedM1(ADDR, 0)

    # Phase 2: forward at full speed until back to start (or timeout)
    t1 = time.time()
    rc.SpeedM1(ADDR, abs(target_full_fwd_cps))
    try:
        while True:
            now = (time.time() - t0)  # continue same time base on plot
            ok_s, spd = read_speed()
            if ok_s:
                times.append(now)
                speeds.append(spd)
                refs.append(abs(target_full_fwd_cps))
            # stop condition based on encoder, if available
            if enc_start is not None:
                ok_e, enc_now = read_enc()
                if ok_e and abs(enc_now - enc_start) <= RETURN_TOL:
                    print("Return-to-start reached (within tolerance).")
                    break
            # safety timeout
            if time.time() - t1 > RETURN_TIMEOUT:
                print("WARN: Return timeout reached; stopping.")
                break
            time.sleep(SAMPLE_DT)
    finally:
        rc.SpeedM1(ADDR, 0)

    # Save plot of both phases
    save_speed_plot(times, speeds, refs)

def action_hold(pin, target_cps):
    """Drive at target_cps while the button is held (debounced, minimal jitter)."""
    label = "Forward" if target_cps > 0 else "Backward"
    print(f"Button {label}: hold to run (full speed)")
    rc.SpeedM1(ADDR, target_cps)
    try:
        while GPIO.input(pin) == GPIO.LOW:
            time.sleep(0.02)  # closed-loop runs on RoboClaw; keep loop light
    finally:
        rc.SpeedM1(ADDR, 0)
        wait_for_release_stable(pin)

# ------------- Main -------------
def main():
    # Setup GPIO (active-low with internal pull-ups; wire other side to GND)
    GPIO.setmode(GPIO.BCM)
    for pin in (BTN_SEQ_BACK_THEN_RETURN, BTN_HOLD_BACKWARD, BTN_HOLD_FORWARD):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Connect RoboClaw
    if rc.Open() == 0:
        GPIO.cleanup()
        raise RuntimeError(f"Could not open {PORT}")

    # Version (optional)
    ok, ver = rc.ReadVersion(ADDR)
    print("Version:", ver.decode("utf-8", "ignore") if ok and hasattr(ver, "decode") else ver)

    # Determine QPPS
    qpps = QPPS
    if qpps is None or qpps <= 0:
        print("Measuring QPPS...")
        qpps = measure_qpps()
        if qpps <= 0:
            qpps = 5000
            print("QPPS measurement failed; using fallback =", qpps)
    print("QPPS =", qpps)

    # Configure velocity PID
    rc.SetM1VelocityPID(ADDR, fix16(KP), fix16(KI), fix16(KD), int(qpps))

    # Targets
    target_slow     = int(TARGET_FRACTION * qpps)           # slow speed (button 1 backward)
    target_full_fwd = int(FULL_SPEED_FRACTION * qpps)       # full forward (return + button 3)
    target_full_rev = -int(FULL_SPEED_FRACTION * qpps)      # full backward (button 2)

    print("Targets (cps): slow =", target_slow,
          " full_fwd =", target_full_fwd, " full_rev =", target_full_rev)
    print("Ready. Press:")
    print("  * GPIO17: Backward 30s (slow) → Forward to starting encoder (full) + save plot")
    print("  * GPIO27: Backward while held (full)")
    print("  * GPIO22: Forward while held (full)")
    print("Press Ctrl+C to quit.")

    try:
        while True:
            # Button 1: backward 30s then forward to start (with plot)
            if GPIO.input(BTN_SEQ_BACK_THEN_RETURN) == GPIO.LOW:
                if wait_for_press_stable(BTN_SEQ_BACK_THEN_RETURN, timeout=0.5):
                    action_seq_back_then_return(target_slow, target_full_fwd)
                    wait_for_release_stable(BTN_SEQ_BACK_THEN_RETURN)

            # Button 2 (SWAPPED): backward while held (full)
            if GPIO.input(BTN_HOLD_BACKWARD) == GPIO.LOW:
                if wait_for_press_stable(BTN_HOLD_BACKWARD, timeout=0.5):
                    action_hold(BTN_HOLD_BACKWARD, target_full_rev)

            # Button 3 (SWAPPED): forward while held (full)
            if GPIO.input(BTN_HOLD_FORWARD) == GPIO.LOW:
                if wait_for_press_stable(BTN_HOLD_FORWARD, timeout=0.5):
                    action_hold(BTN_HOLD_FORWARD, target_full_fwd)

            time.sleep(0.01)  # main loop idle
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rc.SpeedM1(ADDR, 0)
            rc.ForwardM1(ADDR, 0)
        except Exception:
            pass
        GPIO.cleanup()
        print("Stopped and cleaned up.")

if __name__ == "__main__":
    main()
