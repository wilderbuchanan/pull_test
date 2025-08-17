# m1_speed_hold_cli.py
import os
import time
from roboclaw_3 import Roboclaw

# --- headless-safe plotting backend (no GUI needed) ---
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PORT  = "/dev/ttyACM0"
BAUD  = 38400
ADDR  = 0x80

# Put your measured QPPS here to skip re-measure; else set to None
QPPS = 4196   # <-- replace with your measured value (or None)

# Start conservative; tune later
KP, KI, KD = 0.35, 0.007, 0.04

TARGET_FRACTION = 0.40           # target = 40% of QPPS
SAMPLE_DT = 0.10                 # seconds between prints
RUN_SECONDS = 20                 # run time

rc = Roboclaw(PORT, BAUD)

def fix16(x: float) -> int:
    return int(x * 65536.0)

def read_speed():
    out = rc.ReadSpeedM1(ADDR)
    if isinstance(out, tuple) and len(out) >= 2:
        return bool(out[0]), int(out[1])
    return False, 0

def read_enc():
    out = rc.ReadEncM1(ADDR)
    if isinstance(out, tuple) and len(out) >= 2:
        return bool(out[0]), int(out[1])
    return False, 0

def measure_qpps(seconds=1.5):
    samples = []
    def grab(sec, fwd=True):
        if fwd:
            rc.ForwardM1(ADDR, 127)
        else:
            rc.BackwardM1(ADDR, 127)
        t0 = time.time()
        while time.time() - t0 < sec:
            ok, s = read_speed()
            if ok: samples.append(abs(s))
            time.sleep(0.05)
        rc.ForwardM1(ADDR, 0)
    grab(seconds, True)
    time.sleep(0.2)
    grab(seconds, False)
    return max(samples) if samples else 0

def next_increment_path(dirpath: str, basename: str, ext: str = ".png") -> str:
    os.makedirs(dirpath, exist_ok=True)
    i = 1
    while True:
        candidate = os.path.join(dirpath, f"{basename}_{i}{ext}")
        if not os.path.exists(candidate):
            return candidate
        i += 1

def save_relative_plot(times, speeds, refs, out_dir="output_photos"):
    # speed relative to reference (error)
    rel = [s - r for s, r in zip(speeds, refs)]
    path = next_increment_path(out_dir, "speed_relative_to_reference", ".png")

    plt.figure(figsize=(10, 6))
    plt.plot(times, rel, label="Speed - Reference")
    plt.axhline(0, linestyle="--", label="Reference (0)")
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Speed (counts/sec)")
    plt.title("Motor Speed Relative to Reference")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved plot: {path}")

def main():
    if rc.Open() == 0:
        raise RuntimeError(f"Could not open {PORT}")

    ok, ver = rc.ReadVersion(ADDR)
    print("Version:", ver.decode("utf-8", "ignore") if ok and hasattr(ver, "decode") else ver)

    qpps = QPPS
    if qpps is None or qpps <= 0:
        print("Measuring QPPS...")
        qpps = measure_qpps()
        if qpps <= 0:
            qpps = 5000
            print("QPPS measurement failed; using fallback =", qpps)
    print("QPPS =", qpps)

    # Set velocity PID and QPPS
    rc.SetM1VelocityPID(ADDR, fix16(KP), fix16(KI), fix16(KD), int(qpps))

    target = int(TARGET_FRACTION * qpps)
    print("Target speed (counts/sec) =", target)

    # Zero encoder (optional)
    try:
        rc.ResetEncoders(ADDR)
    except AttributeError:
        try: rc.SetEncM1(ADDR, 0)
        except Exception: pass

    # Command closed-loop speed
    rc.SpeedM1(ADDR, target)

    print("\n   t(s)   speed(cps)    err   enc(counts)")

    # --- collect data for plotting ---
    times, speeds, refs = [], [], []

    t0 = time.time()
    try:
        while time.time() - t0 < RUN_SECONDS:
            now = time.time() - t0
            ok_s, spd = read_speed()
            ok_e, enc = read_enc()
            if ok_s:
                err = target - spd
                print(f"{now:7.2f}  {spd:10d}  {err:6d}  {enc:10d}")

                # store for plot
                times.append(now)
                speeds.append(spd)
                refs.append(target)  # constant reference (could vary if you change target)
            else:
                print(f"{now:7.2f}  (speed read failed)")
            time.sleep(SAMPLE_DT)
    except KeyboardInterrupt:
        pass
    finally:
        rc.SpeedM1(ADDR, 0)
        rc.ForwardM1(ADDR, 0)
        print("Stopped.")
        # save plot at the end
        if times and speeds and refs:
            save_relative_plot(times, speeds, refs)

if __name__ == "__main__":
    main()
