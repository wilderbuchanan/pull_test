#!/usr/bin/env python3
"""
HP/MXmoon Force Gauge (Modbus RTU @9600 8N1) — max throughput
- Requests ONLY the 2 regs holding live force (big-endian float at regs[0..1])
- Free-runs (no pacing) by default to maximize sample rate
- Silent per-sample (no prints) unless --debug is used
- Keys: 'z' tare (sets current reading as zero), 'q' quit
- Prefilled test name prompt; saves to ./data/<name>.csv
- CSV columns: t_rel_s, force_N, force_kgf, sample
"""

import os, re, sys, time, struct, argparse, select, termios, tty
from pathlib import Path
import serial

# ---------- Modbus / device ----------
SLAVE_ADDR   = 1
FUNC_READ_HR = 0x03
START_ADDR   = 0x0000     # live force starts at 0x0000
COUNT_REGS   = 2          # ONLY 2 registers (live force)
BAUD         = 9600

LIVE_PAIR_INDEX = 0       # regs[0..1]
N_TO_KGF = 1.0 / 9.80665  # 1 kgf = 9.80665 N

# ---------- CRC & framing ----------
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

def build_req(slave=SLAVE_ADDR, start=START_ADDR, count=COUNT_REGS) -> bytes:
    pdu = bytes([slave, FUNC_READ_HR,
                 (start >> 8) & 0xFF, start & 0xFF,
                 (count >> 8) & 0xFF, count & 0xFF])
    c = crc16_modbus(pdu)
    return pdu + bytes([c & 0xFF, (c >> 8) & 0xFF])

# ---------- fast read helpers ----------
def read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    """Read exactly n bytes, returning empty on timeout (uses a tight loop)."""
    buf = bytearray()
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        # No sleep: we want max throughput; rely on serial driver scheduling
    return bytes(buf) if len(buf) == n else b""

# ---------- terminal hotkeys ----------
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

# ---------- name helpers ----------
def input_with_prefill(prompt: str, prefill: str) -> str:
    try:
        import readline
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
    base = re.sub(r"\d+$", "", last_name)
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

# ---------- main ----------
def main():
    ap = argparse.ArgumentParser(description="Max-speed force logger (silent per-sample).")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    ap.add_argument("--count", type=int, default=COUNT_REGS, help="Number of regs to read (default 2)")
    ap.add_argument("--debug", action="store_true", help="Print periodic stats")
    ap.add_argument("--debug-interval", type=float, default=2.0, help="Seconds between debug prints")
    args = ap.parse_args()

    # Prefilled sample name
    last_file = Path(".last_test_name.txt")
    last_name = last_file.read_text(encoding="utf-8").strip() if last_file.exists() else ""
    default_name = suggest_from_last(last_name)
    sample_name = input_with_prefill("Sample name: ", default_name).strip() or default_name
    try:
        last_file.write_text(sample_name, encoding="utf-8")
    except Exception:
        pass

    # CSV
    csv_path = choose_csv_path(sample_name)

    # Prebuild request
    req = build_req(slave=SLAVE_ADDR, start=START_ADDR, count=max(2, args.count))

    # Open serial
    # Keep timeouts short to avoid blocking; we use our own deadlines in read_exact.
    ser = serial.Serial(
        args.port, BAUD,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        timeout=0.0,              # nonblocking reads; we'll enforce deadlines ourselves
        write_timeout=0.05,
        inter_byte_timeout=None
    )
    ser.setDTR(True); ser.setRTS(True)
    time.sleep(0.03)
    ser.reset_input_buffer(); ser.reset_output_buffer()

    kg = KeyGetter()
    tare_N = 0.0
    samples = 0
    t0 = time.perf_counter()
    last_dbg = t0

    # Open file once; flush occasionally for safety (without fsync to keep speed)
    f = open(csv_path, "w", encoding="utf-8")
    f.write("t_rel_s,force_N,force_kgf,sample\n")
    last_flush = time.time()

    try:
        while True:
            # send request
            ser.write(req)

            # read response header (3 bytes): addr, func, byte_count
            deadline = time.time() + 0.05  # ~50ms max wait per transaction
            hdr = read_exact(ser, 3, deadline)
            if not hdr or len(hdr) != 3 or hdr[0] != SLAVE_ADDR or hdr[1] != FUNC_READ_HR:
                # check keys and continue (don’t print per-sample)
                k = kg.get()
                if k in ("q","Q"): break
                if k in ("z","Z"): tare_N = 0.0  # clear; next line will set tare to current
                continue

            bc = hdr[2]
            # read bc data + 2 CRC
            body = read_exact(ser, bc + 2, time.time() + 0.05)
            if len(body) != bc + 2:
                k = kg.get()
                if k in ("q","Q"): break
                if k in ("z","Z"): tare_N = 0.0
                continue

            # CRC check
            rx_crc = body[-2] | (body[-1] << 8)
            if rx_crc != crc16_modbus(hdr + body[:-2]):
                continue

            # decode big-endian float from regs[0..1]
            data = body[:-2]  # bc bytes
            if bc < 4:
                continue
            reg0 = (data[0] << 8) | data[1]
            reg1 = (data[2] << 8) | data[3]
            be = struct.pack(">HH", reg0, reg1)
            raw_N = struct.unpack(">f", be)[0]

            # hotkeys (tare uses the *current* raw reading)
            k = kg.get()
            if k in ("q","Q"):
                break
            if k in ("z","Z"):
                tare_N = raw_N  # set zero at current reading

            force_N = raw_N - tare_N
            force_kgf = force_N * N_TO_KGF
            t_rel = time.perf_counter() - t0

            # write line (no per-sample prints)
            f.write(f"{t_rel:.6f},{force_N:.6f},{force_kgf:.6f},{sample_name}\n")
            samples += 1

            # occasional lightweight flush
            now = time.time()
            if now - last_flush >= 1.0:
                f.flush()
                last_flush = now

            # optional debug summary
            if args.debug and (time.perf_counter() - last_dbg) >= args.debug_interval:
                elapsed = time.perf_counter() - t0
                hz = samples / elapsed if elapsed > 0 else 0.0
                print(f"[{time.strftime('%H:%M:%S')}] samples={samples} elapsed={elapsed:.2f}s avg={hz:.1f} Hz")
                last_dbg = time.perf_counter()

    except KeyboardInterrupt:
        pass
    finally:
        try: kg.restore()
        except: pass
        try:
            f.flush(); f.close()
        except: pass
        try:
            ser.close()
        except: pass

        elapsed = time.perf_counter() - t0
        hz = samples / elapsed if elapsed > 0 else 0.0
        print(f"[{time.strftime('%H:%M:%S')}] Done. Saved {samples} samples to {csv_path} "
              f"(elapsed {elapsed:.2f}s, avg {hz:.1f} Hz).")

if __name__ == "__main__":
    main()
