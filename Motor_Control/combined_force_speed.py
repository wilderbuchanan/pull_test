#!/usr/bin/env python3
# combined_controller.py
# Button 1 (stateful):
#   A) Press 1: save current encoder -> move to (start ± 150000) counts (auto-detected sign)
#      Logs CSV+PNG; if force >10 N and later <1 N, finish remainder at FULL-speed backward.
#      Stop when within ±100 of goal OR total travel ≥ 150000-100 from start. Print final encoder.
#      NEW: light/heavy PID switching based on force (hysteresis) for smooth no-load + strong under load.
#   B) Press 1 again: return to saved encoder (±25 tol) -> clear saved position
# Buttons 2/3: full-speed backward/forward while held.

import os, sys, time, argparse, threading, csv, re
from pathlib import Path

def ts_stamp():
    return time.strftime("%Y%m%d-%H%M%S")

# =========================
# -------- MOTOR ----------
# =========================
def motor_main():
    # Lazy imports so it still runs on the Pi cleanly
    try:
        import RPi.GPIO as GPIO
    except ModuleNotFoundError:
        sys.stderr.write(
            "\nERROR: RPi.GPIO not found.\n"
            "Install:\n"
            "  sudo apt-get update && sudo apt-get install -y python3-rpi.gpio\n"
            "If in a virtualenv, also: pip install RPi.GPIO\n\n"
        )
        sys.exit(1)

    from roboclaw_3 import Roboclaw
    import matplotlib
    matplotlib.use("Agg")  # headless-friendly
    import matplotlib.pyplot as plt
    import serial, struct, threading as _threading

    # ---- User settings ----
    PORT  = "/dev/ttyACM0"
    BAUD  = 38400
    ADDR  = 0x80

    QPPS  = 4196                    # set to None to auto-measure

    # Keep your "good under load" gains as the heavy set
    PID_LIGHT = (0.30, 0.004, 0.06)  # smoother at low load
    PID_HEAVY = (0.55, 0.020, 0.02)  # strong tracking under load

    # Force-based PID switch hysteresis
    HEAVY_THRESH  = 5.0    # N: switch to heavy if force stays above this
    LIGHT_THRESH  = 3.0    # N: switch back to light if force stays below this
    HEAVY_ON_DUR  = 0.5    # seconds above threshold to switch to heavy
    LIGHT_ON_DUR  = 1.5    # seconds below threshold to switch back to light

    TARGET_FRACTION     = 0.60      # slow speed (button 1 retract)
    FULL_SPEED_FRACTION = 0.95      # full speed for hold buttons & return

    # Pins (BCM)
    BTN_STATEFUL_1      = 17   # Button 1 (state machine)
    BTN_HOLD_BACKWARD   = 27   # Button 2 (backward while held)
    BTN_HOLD_FORWARD    = 22   # Button 3 (forward  while held)

    # Timing / debounce
    SAMPLE_DT        = 0.02   # 50 Hz fixed-rate sampling while logging
    DEBOUNCE_MS      = 30
    RETURN_TOL       = 25     # counts: acceptable range around start encoder (return phase)
    RETURN_TIMEOUT   = 60     # seconds safety for return phase
    NEAR_BAND        = 200    # counts: switch to slow speed when inside this band (return phase)

    # Position targets for Button 1 slow phase
    RETRACT_DELTA    = 150000  # counts to move from the start encoder (sign auto-detected)
    RETRACT_TOL      = 100     # counts: acceptable range around target goal
    SLOW_PHASE_TIMEOUT = 300   # ultimate safety (s) so it can’t run forever

    # Yield time inside force thread (prevents starving speed loop)
    FORCE_LOOP_SLEEP = 0.002  # ~2 ms

    # Output directory
    OUTPUT_DIR = Path("./output_photos")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Force logger (Modbus) settings
    FORCE_PORT   = "/dev/ttyUSB0"
    FORCE_BAUD   = 9600
    MODBUS_SLAVE = 1
    MODBUS_FUNC  = 0x03
    MODBUS_START = 0x0000
    MODBUS_COUNT = 2

    rc = Roboclaw(PORT, BAUD)

    # ---- Helpers ----
    def fix16(x: float) -> int:
        return int(x * 65536.0)

    def set_velocity_pid(kp, ki, kd, qpps):
        rc.SetM1VelocityPID(ADDR, fix16(kp), fix16(ki), fix16(kd), int(qpps))

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

    # --- Incrementing sample prefix (shared PNG/CSV name) ---
    def next_sample_basepath(out_dir: Path) -> Path:
        max_n = 0
        pattern = re.compile(r"^(\d+)_")
        for p in out_dir.iterdir():
            m = pattern.match(p.name)
            if m:
                try:
                    n = int(m.group(1)); max_n = max(max_n, n)
                except ValueError:
                    pass
        nxt = max_n + 1
        base = f"{nxt:03d}_{ts_stamp()}"
        return out_dir / base  # caller adds .csv / .png

    def save_combined_png(png_fullpath: Path, times, speeds, refs, csv_path: Path):
        force_t, force_N = [], []
        try:
            with open(csv_path, "r", encoding="utf-8") as f:
                r = csv.reader(f); _ = next(r, None)
                for row in r:
                    if len(row) >= 2:
                        try:
                            force_t.append(float(row[0])); force_N.append(float(row[1]))
                        except ValueError:
                            pass
        except FileNotFoundError:
            print(f"NOTE: CSV not found at plot time ({csv_path}); plotting only speed.")

        png_fullpath.parent.mkdir(parents=True, exist_ok=True)
        plt.figure(figsize=(10, 8))

        ax1 = plt.subplot(2, 1, 1)
        ax1.plot(times, speeds, label="Measured Speed")
        ax1.plot(times, refs, "--", label="Reference Speed")
        ax1.set_ylabel("Speed (counts/sec)")
        ax1.set_title("Speed vs Reference (Slow Phase Only)")
        ax1.grid(True); ax1.legend(loc="best")

        ax2 = plt.subplot(2, 1, 2, sharex=None)
        if force_t and force_N:
            ax2.plot(force_t, force_N, label="Force (N)")
            ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Force (N)")
            ax2.set_title("Force vs Time (N)")
            ax2.grid(True); ax2.legend(loc="best")
        else:
            ax2.text(0.5, 0.5, "No force data", ha="center", va="center", transform=ax2.transAxes)
            ax2.axis("off")

        plt.tight_layout(); plt.savefig(str(png_fullpath), dpi=150); plt.close()
        print(f"Saved combined figure: {png_fullpath}")

    def measure_qpps(seconds=1.5):
        samples = []
        def sample_for(sec, forward=True):
            if forward: rc.ForwardM1(ADDR, 127)
            else:       rc.BackwardM1(ADDR, 127)
            t0 = time.time()
            while time.time() - t0 < sec:
                ok, s = read_speed()
                if ok: samples.append(abs(s))
                time.sleep(0.05)
            rc.ForwardM1(ADDR, 0)
        sample_for(seconds, True); time.sleep(0.2); sample_for(seconds, False)
        return max(samples) if samples else 0

    def wait_for_press_stable(pin, timeout=None):
        t_start = time.time()
        while True:
            if GPIO.input(pin) == GPIO.LOW:
                time.sleep(DEBOUNCE_MS / 1000.0)
                if GPIO.input(pin) == GPIO.LOW: return True
            if timeout is not None and (time.time() - t_start) > timeout: return False
            time.sleep(0.005)

    def wait_for_release_stable(pin):
        while True:
            if GPIO.input(pin) == GPIO.HIGH:
                time.sleep(DEBOUNCE_MS / 1000.0)
                if GPIO.input(pin) == GPIO.HIGH: return
            time.sleep(0.005)

    # ---- Minimal Modbus helpers (force thread) ----
    def crc16_modbus(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                lsb = crc & 1; crc >>= 1
                if lsb: crc ^= 0xA001
        return crc

    def build_req(slave=MODBUS_SLAVE, start=MODBUS_START, count=MODBUS_COUNT) -> bytes:
        pdu = bytes([slave, MODBUS_FUNC,
                     (start >> 8) & 0xFF, start & 0xFF,
                     (count >> 8) & 0xFF, count & 0xFF])
        c = crc16_modbus(pdu)
        return pdu + bytes([c & 0xFF, (c >> 8) & 0xFF])

    def read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
        buf = bytearray()
        while len(buf) < n and time.time() < deadline:
            chunk = ser.read(n - len(buf))
            if chunk: buf.extend(chunk)
        return bytes(buf) if len(buf) == n else b""

    class ForceThread(threading.Thread):
        """Logs force; auto-tares; exposes latest force & 'saw_over_10N' flag."""
        def __init__(self, csv_fullpath: Path, port=FORCE_PORT, baud=FORCE_BAUD):
            super().__init__(daemon=True)
            self.port = port; self.baud = baud
            self.csv_path = Path(csv_fullpath); self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.stop_event = threading.Event()
            self.samples = 0; self._exc = None
            self.latest_force_N = None; self.saw_over_10 = False
            self._lock = _threading.Lock()

        def run(self):
            try:
                ser = serial.Serial(self.port, self.baud,
                                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                    timeout=0.0, write_timeout=0.05)
                ser.setDTR(True); ser.setRTS(True); time.sleep(0.03)
                ser.reset_input_buffer(); ser.reset_output_buffer()

                req = build_req()
                f = open(self.csv_path, "w", encoding="utf-8")
                f.write("t_rel_s,force_N,force_kgf\n")
                N_TO_KGF = 1.0 / 9.80665

                t0 = time.perf_counter()
                tare_samples, tare_N = [], 0.0

                while not self.stop_event.is_set():
                    ser.write(req)
                    hdr = read_exact(ser, 3, time.time() + 0.05)
                    if len(hdr) != 3 or hdr[0] != MODBUS_SLAVE or hdr[1] != MODBUS_FUNC:
                        time.sleep(FORCE_LOOP_SLEEP); continue
                    bc = hdr[2]
                    body = read_exact(ser, bc + 2, time.time() + 0.05)
                    if len(body) != bc + 2: time.sleep(FORCE_LOOP_SLEEP); continue
                    rx_crc = body[-2] | (body[-1] << 8)
                    if rx_crc != crc16_modbus(hdr + body[:-2]): time.sleep(FORCE_LOOP_SLEEP); continue
                    data = body[:-2]
                    if bc < 4: time.sleep(FORCE_LOOP_SLEEP); continue

                    reg0 = (data[0] << 8) | data[1]
                    reg1 = (data[2] << 8) | data[3]
                    be = struct.pack(">HH", reg0, reg1)
                    raw_N = struct.unpack(">f", be)[0]

                    if len(tare_samples) < 10:
                        tare_samples.append(raw_N)
                        tare_N = sum(tare_samples)/len(tare_samples)

                    force_N = raw_N - tare_N
                    force_kgf = force_N * N_TO_KGF
                    t_rel = time.perf_counter() - t0

                    f.write(f"{t_rel:.6f},{force_N:.6f},{force_kgf:.6f}\n")
                    self.samples += 1
                    if self.samples % 200 == 0: f.flush()

                    with self._lock:
                        self.latest_force_N = force_N
                        if force_N > 10.0: self.saw_over_10 = True

                    time.sleep(FORCE_LOOP_SLEEP)

                try: f.flush(); f.close()
                except: pass
                try: ser.close()
                except: pass
            except Exception as e:
                self._exc = e

        def snapshot(self):
            with self._lock:
                return (self.latest_force_N, self.saw_over_10)

    # ---------- Stateful Button-1 actions ----------
    saved_start_enc = None   # None -> next Button 1 press starts a new slow retract
    busy = False             # simple guard to avoid re-entrance

    def do_slow_retract_and_log(target_slow_cps):
        """Move from start encoder by RETRACT_DELTA counts (sign auto-detected) with logging.
           Switch to full-speed for remainder if force rule triggers.
           NEW: PID switches between LIGHT/HEAVY based on measured force (with hysteresis)."""
        nonlocal saved_start_enc
        print("BTN1: START position-based retract/logging.")

        # Capture starting encoder
        ok_e0, enc_start = read_enc()
        if not ok_e0:
            print("WARN: Could not read starting encoder; aborting this cycle.")
            return
        saved_start_enc = enc_start

        # Prepare output paths with incrementing sample number
        basepath = next_sample_basepath(OUTPUT_DIR)  # e.g., 003_2025...
        csv_path = basepath.with_suffix(".csv")
        png_path = basepath.with_suffix(".png")

        # Start force logging (ONLY for this move)
        ft = ForceThread(csv_fullpath=csv_path)
        ft.start()
        print(f"[force] logging to {csv_path}")

        # Logging arrays (for PNG top subplot)
        times, speeds, refs = [], [], []

        # Commanded speeds
        full_rev_cps = -int(FULL_SPEED_FRACTION * (QPPS if QPPS else 5000))
        slow_rev_cps = -abs(target_slow_cps)

        # Begin move at slow speed (backward command)
        t0 = time.time()
        rc.SpeedM1(ADDR, slow_rev_cps)
        running_full = False
        next_tick = t0

        # --- PID mode management (start LIGHT for no-load smoothness) ---
        set_velocity_pid(*PID_LIGHT, QPPS if QPPS else 5000)
        mode = "light"
        above_since = None
        below_since = time.time()

        # ---- Auto-detect encoder sign w.r.t. backward command ----
        enc_goal = enc_start - RETRACT_DELTA   # tentative (negative direction)
        sign_checked = False
        sign_window_end = t0 + 0.2  # ~200 ms window to detect sign

        try:
            while True:
                now = time.time()

                # Safety timeout
                if now - t0 > SLOW_PHASE_TIMEOUT:
                    print("WARN: Slow phase timeout reached; stopping.")
                    break

                # Encoder read & goal logic
                ok_e, enc_now = read_enc()
                if ok_e:
                    if not sign_checked and now >= sign_window_end:
                        delta = enc_now - enc_start
                        if delta > 0:
                            enc_goal = enc_start + RETRACT_DELTA
                            print(f"Auto-detected: backward command increases counts; goal corrected to {enc_goal}")
                        else:
                            print(f"Auto-detected: backward command decreases counts; goal kept at {enc_goal}")
                        sign_checked = True

                    if abs(enc_now - enc_goal) <= RETRACT_TOL:
                        print(f"Reached goal band: enc={enc_now}, goal={enc_goal}, tol=±{RETRACT_TOL}")
                        break

                    if abs(enc_now - enc_start) >= (RETRACT_DELTA - RETRACT_TOL):
                        print(f"Reached travel threshold from start: enc={enc_now}, start={enc_start}, "
                              f"travel≈{abs(enc_now - enc_start)} ≥ {RETRACT_DELTA - RETRACT_TOL}")
                        break

                # ---- Force-based PID switching (hysteresis) ----
                latest_force, saw_over_10 = ft.snapshot()
                if latest_force is not None:
                    if latest_force > HEAVY_THRESH:
                        above_since = above_since or now
                        below_since = None
                    elif latest_force < LIGHT_THRESH:
                        below_since = below_since or now
                        above_since = None

                    if mode == "light" and above_since and (now - above_since) >= HEAVY_ON_DUR:
                        set_velocity_pid(*PID_HEAVY, QPPS if QPPS else 5000)
                        mode = "heavy"
                        print(f"[PID] → HEAVY (force {latest_force:.1f} N)")
                    elif mode == "heavy" and below_since and (now - below_since) >= LIGHT_ON_DUR:
                        set_velocity_pid(*PID_LIGHT, QPPS if QPPS else 5000)
                        mode = "light"
                        print(f"[PID] → LIGHT (force {latest_force:.1f} N)")

                # ---- Force-drop rule: switch commanded speed to FULL for remainder ----
                if (not running_full) and saw_over_10 and (latest_force is not None) and (latest_force < 1.0):
                    print("Force drop condition met (>10N seen, now <1N) — switching to FULL-SPEED retract for remainder.")
                    rc.SpeedM1(ADDR, full_rev_cps)
                    running_full = True

                # Log speed/reference each tick
                ok_s, spd = read_speed()
                if ok_s:
                    t_rel = now - t0
                    times.append(t_rel)
                    speeds.append(spd)
                    refs.append(full_rev_cps if running_full else slow_rev_cps)

                # precise cadence
                next_tick += SAMPLE_DT
                sleep_for = next_tick - time.time()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            rc.SpeedM1(ADDR, 0)

        # Print encoder at conclusion
        ok_e_end, enc_end = read_enc()
        if ok_e_end:
            print(f"Slow phase end encoder: {enc_end}")
        else:
            print("Slow phase end encoder: (read failed)")

        # Stop force logger and save combined PNG
        ft.stop_event.set()
        ft.join(timeout=2.0)
        if ft._exc:
            print(f"[force] ERROR: {ft._exc}")
        else:
            print(f"[force] done, samples={ft.samples}, file={csv_path}")
        save_combined_png(png_fullpath=png_path, times=times, speeds=speeds, refs=refs, csv_path=csv_path)

        print("BTN1: move complete. System idle; Button 1 now = RETURN to saved start.")

    def do_return_to_saved(target_full_fwd_cps):
        """Drive back to saved_start_enc with two-stage fast/slow; clear when done."""
        nonlocal saved_start_enc
        if saved_start_enc is None:
            print("BTN1: No saved start position. Press Button 1 to start a new retract.")
            return

        print(f"BTN1: RETURN to saved encoder {saved_start_enc} (±{RETURN_TOL}).")

        FAST = abs(target_full_fwd_cps)
        SLOW = max(200, int(0.3 * FAST))

        def command_dir_speed(err):
            if err > 0:
                rc.SpeedM1(ADDR, FAST if err > NEAR_BAND else SLOW)
            elif err < 0:
                rc.SpeedM1(ADDR, -FAST if err < -NEAR_BAND else -SLOW)
            else:
                rc.SpeedM1(ADDR, 0)

        rc.SpeedM1(ADDR, FAST)
        try:
            t_return_start = time.time()
            while True:
                ok_e, enc_now = read_enc()
                if ok_e:
                    err = saved_start_enc - enc_now  # + => need forward
                    if abs(err) <= RETURN_TOL:
                        print(f"Return OK: enc_now={enc_now}, target={saved_start_enc}, |err|={abs(err)}<= {RETURN_TOL}")
                        break
                    command_dir_speed(err)
                if time.time() - t_return_start > RETURN_TIMEOUT:
                    print("WARN: Return timeout reached; stopping.")
                    break
                time.sleep(0.01)
        finally:
            rc.SpeedM1(ADDR, 0)

        saved_start_enc = None
        print("BTN1: return complete. Next Button-1 press will start a new retract.")

    def action_hold(pin, target_cps):
        label = "Forward" if target_cps > 0 else "Backward"
        print(f"Button {label}: hold to run (full speed)")
        rc.SpeedM1(ADDR, target_cps)
        try:
            while GPIO.input(pin) == GPIO.LOW:
                time.sleep(0.02)
        finally:
            rc.SpeedM1(ADDR, 0)
            wait_for_release_stable(pin)

    # ---- Setup GPIO ----
    GPIO.setmode(GPIO.BCM)
    for pin in (BTN_STATEFUL_1, BTN_HOLD_BACKWARD, BTN_HOLD_FORWARD):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # ---- Connect RoboClaw ----
    if rc.Open() == 0:
        GPIO.cleanup()
        raise RuntimeError(f"Could not open {PORT}")

    ok, ver = rc.ReadVersion(ADDR)
    print("Version:", ver.decode("utf-8", "ignore") if ok and hasattr(ver, "decode") else ver)

    # ---- QPPS ----
    qpps = QPPS
    if qpps is None or qpps <= 0:
        print("Measuring QPPS...")
        qpps = measure_qpps()
        if qpps <= 0:
            qpps = 5000
            print("QPPS measurement failed; using fallback =", qpps)
    print("QPPS =", qpps)

    # ---- PID (initialize to LIGHT so idle/no‑load is calm) ----
    set_velocity_pid(*PID_LIGHT, qpps)

    # ---- Targets ----
    target_slow     = int(TARGET_FRACTION * qpps)
    target_full_fwd = int(FULL_SPEED_FRACTION * qpps)
    target_full_rev = -int(FULL_SPEED_FRACTION * qpps)

    print("Targets (cps): slow =", target_slow,
          " full_fwd =", target_full_fwd, " full_rev =", target_full_rev)
    print("Ready. Press:")
    print("  * GPIO17: State A -> move by 150000 counts (auto sign, ±100) + log (force-drop → fast; PID auto-swap)")
    print("            State B -> return to saved encoder (±25)")
    print("  * GPIO27: Backward while held (full)")
    print("  * GPIO22: Forward while held (full)")
    print("Press Ctrl+C to quit.")

    try:
        while True:
            if GPIO.input(BTN_STATEFUL_1) == GPIO.LOW and not busy:
                if wait_for_press_stable(BTN_STATEFUL_1, timeout=0.5):
                    busy = True
                    try:
                        if saved_start_enc is None:
                            do_slow_retract_and_log(target_slow)
                        else:
                            do_return_to_saved(target_full_fwd)
                    finally:
                        busy = False
                    wait_for_release_stable(BTN_STATEFUL_1)

            if GPIO.input(BTN_HOLD_BACKWARD) == GPIO.LOW and not busy:
                if wait_for_press_stable(BTN_HOLD_BACKWARD, timeout=0.5):
                    action_hold(BTN_HOLD_BACKWARD, target_full_rev)

            if GPIO.input(BTN_HOLD_FORWARD) == GPIO.LOW and not busy:
                if wait_for_press_stable(BTN_HOLD_FORWARD, timeout=0.5):
                    action_hold(BTN_HOLD_FORWARD, target_full_fwd)

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rc.SpeedM1(ADDR, 0); rc.ForwardM1(ADDR, 0)
        except Exception:
            pass
        GPIO.cleanup()
        print("Stopped and cleaned up.")

# =========================
# -------- ENTRY ----------
# =========================
def main():
    parser = argparse.ArgumentParser(description="Stateful motor controller with position-based retract logging and return (PID auto-switch).")
    parser.add_argument("mode", nargs="?", default="motor", choices=["motor"])
    args = parser.parse_args()
    if args.mode == "motor":
        motor_main()

if __name__ == "__main__":
    main()
