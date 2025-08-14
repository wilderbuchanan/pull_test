#!/usr/bin/env python3
"""
HP/MXmoon Force Gauge (Modbus RTU @9600 8N1)
- Polls 13 holding registers (01 03 0000 000D)
- Decodes live force from regs[0..1] as big-endian float (f00) in Newtons
- Prompts for a test sample name with PREFILLED default = last name with trailing digits stripped (e.g., 'test-1' -> 'test-')
- Saves CSV to ./data/<test-name>.csv (adds timestamp if file exists)
- Logs both Newtons and kgf (force_N, force_kgf)
- Keys: 'z' tare, 'q' quit
"""

import os, re, sys, time, struct, argparse, select, termios, tty
from pathlib import Path
import serial

# ---- Modbus request (fixed to your gauge) ----
REQ_SLAVE, REQ_START, REQ_COUNT = 1, 0x0000, 13
LIVE_PAIR_INDEX = 0         # f00 (regs 0..1) is the live force
POLL_HZ = 10.0

# ---- Conversions ----
N_TO_KGF = 1.0 / 9.80665    # 1 kgf = 9.80665 N

# ---- Helpers ----
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc

def build_req(slave=REQ_SLAVE, start=REQ_START, count=REQ_COUNT) -> bytes:
    pdu = bytes([slave, 0x03, (start>>8)&0xFF, start&0xFF, (count>>8)&0xFF, count&0xFF])
    c = crc16_modbus(pdu)
    return pdu + bytes([c & 0xFF, (c>>8) & 0xFF])

def read_exact(ser: serial.Serial, n: int, timeout_s: float=0.5) -> bytes:
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n and (time.time()-t0) < timeout_s:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)

def ts():
    return time.strftime("%H:%M:%S")

# --- Prefilled input on Linux/macOS terminals ---
def input_with_prefill(prompt: str, prefill: str) -> str:
    try:
        import readline  # POSIX only
        def hook():
            readline.insert_text(prefill); readline.redisplay()
        readline.set_startup_hook(hook)
        try:
            return input(prompt)
        finally:
            readline.set_startup_hook()
    except Exception:
        s = input(f"{prompt}[{prefill}] ").strip()
        return s or prefill

def suggest_from_last(last_name: str) -> str:
    if not last_name:
        return "test-"
    base = re.sub(r"\d+$", "", last_name)  # "sample-12" -> "sample-"
    return base if base else (last_name[:-1] if last_name else "test-")

def sanitize_filename(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_") or "test"

def choose_csv_path(sample_name: str) -> Path:
    data_dir = Path("./data"); data_dir.mkdir(parents=True, exist_ok=True)
    base = sanitize_filename(sample_name)
    path = data_dir / f"{base}.csv"
    if path.exists():
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = data_dir / f"{base}-{stamp}.csv"
    return path

class KeyGetter:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
    def get(self):
        r,_,_ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
        return None
    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# ---- Main run ----
def main():
    ap = argparse.ArgumentParser(description="HP force gauge reader with test-name CSV (logs kgf)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--hz", type=float, default=POLL_HZ)
    args = ap.parse_args()

    # Load last test name (for suggestion)
    last_file = Path(".last_test_name.txt")
    last_name = ""
    if last_file.exists():
        try:
            last_name = last_file.read_text(encoding="utf-8").strip()
        except Exception:
            pass
    default_name = suggest_from_last(last_name)

    # Ask user with prefilled default
    sample_name = input_with_prefill("Sample name: ", default_name).strip() or default_name

    # Remember this one for next time
    try:
        last_file.write_text(sample_name, encoding="utf-8")
    except Exception:
        pass

    # CSV path
    csv_path = choose_csv_path(sample_name)
    print(f"[{ts()}] Saving to: {csv_path}")

    # Build request
    req = build_req()
    print(f"[{ts()}] Opening {args.port} @ 9600 8N1; polling {args.hz:.1f} Hz")
    print(f"[{ts()}] Request: {req.hex(' ')} (slave=1 func=0x03 start=0x0000 count=13)")
    print("[keys] 'z' = tare (zero), 'q' = quit")

    tare = 0.0
    header_written = False
    period = 1.0 / max(args.hz, 0.5)

    try:
        with serial.Serial(args.port, 9600, timeout=0.12,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE) as ser:
            ser.setDTR(True); ser.setRTS(True); time.sleep(0.05)
            ser.reset_input_buffer(); ser.reset_output_buffer()
            kg = KeyGetter()

            while True:
                t0 = time.time()
                ser.write(req)

                hdr = read_exact(ser, 3, timeout_s=0.35)
                if len(hdr) != 3 or hdr[0] != 1 or hdr[1] != 0x03:
                    k = kg.get()
                    if k in ("q", "Q"): break
                    if k in ("z", "Z"): tare = 0.0
                    dt = time.time() - t0
                    if dt < period: time.sleep(period - dt)
                    continue

                n = hdr[2]
                body = read_exact(ser, n + 2, timeout_s=0.45)
                if len(body) != n + 2:
                    k = kg.get()
                    if k in ("q", "Q"): break
                    if k in ("z", "Z"): tare = 0.0
                    dt = time.time() - t0
                    if dt < period: time.sleep(period - dt)
                    continue

                # CRC
                rx_crc = body[-2] | (body[-1] << 8)
                calc_crc = crc16_modbus(hdr + body[:-2])
                if rx_crc != calc_crc:
                    dt = time.time() - t0
                    if dt < period: time.sleep(period - dt)
                    continue

                # Decode live force (big-endian float at regs[0..1])
                data = body[:-2]
                regs = [ (data[i]<<8) | data[i+1] for i in range(0, n, 2) ]
                i = max(0, min(LIVE_PAIR_INDEX, (len(regs)//2)-1))
                be = struct.pack(">HH", regs[2*i], regs[2*i+1])
                force_N = struct.unpack(">f", be)[0] - tare
                force_kgf = force_N * N_TO_KGF

                # Print to terminal (still N; add kgf in parentheses)
                print(f"{time.strftime('%H:%M:%S')}  {force_N:.6f}  ({force_kgf:.6f} kgf)")

                # Write CSV (now with kgf)
                if not header_written:
                    csv_path.parent.mkdir(parents=True, exist_ok=True)
                    with open(csv_path, "w", encoding="utf-8") as f:
                        f.write("time,force_N,force_kgf,sample\n")
                    header_written = True
                with open(csv_path, "a", encoding="utf-8") as f:
                    f.write(f"{time.strftime('%H:%M:%S')},{force_N:.6f},{force_kgf:.6f},{sample_name}\n")

                # Hotkeys
                k = kg.get()
                if k in ("q", "Q"):
                    break
                if k in ("z", "Z"):
                    tare = force_N + tare
                    print(f"[{ts()}] TARE applied; offset now {tare:.6f} N")

                # Pace the loop
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            kg.restore()
        except Exception:
            pass
        print(f"[{ts()}] Bye. CSV saved to {csv_path}")

if __name__ == "__main__":
    main()
