#!/usr/bin/env python3
import sys, time, re, serial, binascii, glob

# ---------- Settings ----------
PORTS = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
BAUDS = [9600, 19200]
TIMEOUT = 0.3
POKES = [b"\r", b"\n", b"ONLINE\r", b"READ\r", b"PRINT\r", b"V\r", b"Z\r"]

UNIT_MAP = {
    "N": 1.0, "KN": 1000.0,
    "Kgf": 9.80665, "kgf": 9.80665,
    "lbf": 4.4482216153, "ozf": 0.278013850953
}

# ---------- Helpers ----------
def ts():
    return time.strftime("%H:%M:%S")

def parse_line(text):
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m:
        return None
    try:
        raw_val = float(m.group(1))
    except ValueError:
        return None
    unit = m.group(2)
    factor = UNIT_MAP.get(unit)
    return raw_val, unit, (raw_val * factor if factor else None)

# ---------- Main ----------
def main():
    print(f"[{ts()}] Ports: {PORTS}  Bauds: {BAUDS}")
    for port in PORTS:
        for baud in BAUDS:
            try:
                print(f"[{ts()}] 🔌 Open {port} @ {baud}")
                with serial.Serial(port, baud, timeout=TIMEOUT) as ser:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    print(f"[{ts()}] 👂 Listening (Ctrl+C to stop) … probing for up to 18.0s")
                    t_start = time.time()
                    poke_i = 0
                    buf = bytearray()

                    while time.time() - t_start < 18.0:
                        # every 1s send a poke
                        if (time.time() - t_start) % 1 < TIMEOUT:
                            cmd = POKES[poke_i % len(POKES)]
                            ser.write(cmd)
                            print(f"[{ts()}] TX >> {repr(cmd)}")
                            poke_i += 1

                        data = ser.read(64)
                        if data:
                            hex_str = binascii.hexlify(data).decode()
                            ascii_str = data.decode(errors='replace')
                            print(f"[{ts()}] RX HEX: {hex_str}  ASCII: {ascii_str}")
                            buf.extend(data)
                            while b"\n" in buf or b"\r" in buf:
                                for delim in (b"\n", b"\r"):
                                    if delim in buf:
                                        line, _, rest = buf.partition(delim)
                                        buf = rest
                                        text_line = line.decode(errors='ignore').strip()
                                        if text_line:
                                            parsed = parse_line(text_line)
                                            if parsed:
                                                raw, unit, N = parsed
                                                N_str = "" if N is None else f"{N:.6f}"
                                                safe_line = text_line.replace('"', '""')
                                                print(f"[{ts()}] CSV: {N_str},{raw},{unit},\"{safe_line}\"")

                    print(f"[{ts()}] ⏳ No bytes at {baud} within 18.0s")
            except Exception as e:
                print(f"[{ts()}] ERROR on {port}@{baud}: {e}")
    print(f"[{ts()}] ✅ Scan complete.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped by user.")
