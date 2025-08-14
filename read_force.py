#!/usr/bin/env python3
"""
HP-series force gauge reader (MXmoonfree / new-gen HP)
- Autoscans /dev/ttyUSB* and /dev/ttyACM*
- Tries bauds: 9600, 19200 (configurable)
- Periodically "nudges" device (DTR/RTS dance + common commands)
- Dumps HEX + ASCII, and parses values into Newtons
- Optional CSV logging

Usage examples:
  python3 force_probe_read.py
  python3 force_probe_read.py --bauds 9600 --csv readings.csv --raw
  python3 force_probe_read.py --port /dev/ttyUSB0 --seconds 25 --nudge-every 0.5
"""
import argparse, sys, time, glob, re, binascii, os
import serial

UNIT_MAP = {
    "N": 1.0,
    "kN": 1000.0, "KN": 1000.0,
    "kgf": 9.80665, "Kgf": 9.80665, "gf": 0.00980665,
    "lbf": 4.4482216152605,
    "ozf": 0.278013850953
}

# Commands many gauges respond to (poll/current/print/version/zero etc)
NUDGE_CMDS = [b"\r", b"\n", b"READ\r", b"PRINT\r", b"?\r", b"Q\r", b"R\r", b"P\r", b"S\r", b"D\r", b"V\r"]

def ts():
    return time.strftime("%H:%M:%S")

def hexdump(b: bytes) -> str:
    return binascii.hexlify(b).decode()

def find_ports(globs):
    ports = []
    for g in globs:
        ports.extend(glob.glob(g))
    # de-dupe and sort by device number
    return sorted(set(ports), key=lambda p: (p.rstrip("0123456789"), len(p), p))

def parse_line(text: str):
    """
    Parse a line like:
      '+012.3 N', ' -5.67 lbf', '123.4Kgf', '0.123kN'
    Returns tuple (N_value or None, raw_value, unit) if matched.
    """
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m:
        return None
    val_str, unit = m.group(1), m.group(2)
    try:
        raw = float(val_str)
    except ValueError:
        return None
    # normalize similar-looking units
    u = unit.strip()
    factor = UNIT_MAP.get(u)
    return ((raw * factor) if factor else None, raw, u)

def write_csv_header(path):
    if not os.path.exists(path):
        with open(path, "w", encoding="utf-8") as f:
            f.write("timestamp,port,baud,newtons,raw,unit,ascii_line\n")

def write_csv_row(path, port, baud, N, raw, unit, line):
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{port},{baud},"
                f"{'' if N is None else f'{N:.6f}'},{raw},{unit},\"{line.replace('\"','\"\"')}\"\n")

def try_one(port, baud, seconds, raw_dump, csv_path, nudge_every):
    print(f"[{ts()}] 🔌 Open {port} @ {baud}")
    try:
        with serial.Serial(
            port=port, baudrate=baud, timeout=0.15,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            xonxoff=False, rtscts=False, dsrdtr=False, write_timeout=0.5
        ) as ser:
            # Wake the device a little (some firmwares gate output until host toggles lines)
            ser.setDTR(True);  ser.setRTS(True);  time.sleep(0.05)
            ser.setDTR(False); ser.setRTS(False); time.sleep(0.05)
            ser.setDTR(True);  ser.setRTS(True)
            ser.reset_input_buffer(); ser.reset_output_buffer()

            print(f"[{ts()}] 👂 Listening; probing for up to {seconds:.1f}s (press SEND on gauge once if needed)")
            t0 = time.time()
            last_tx = 0.0
            cmd_idx = 0
            buf = bytearray()
            saw_any = False

            while time.time() - t0 < seconds:
                # Periodic "nudge" / query
                if nudge_every > 0 and (time.time() - last_tx) >= nudge_every:
                    cmd = NUDGE_CMDS[cmd_idx % len(NUDGE_CMDS)]
                    try:
                        ser.write(cmd)
                        print(f"[{ts()}] TX >> {repr(cmd)}", file=sys.stderr)
                    except Exception as e:
                        print(f"[{ts()}] ❌ write error: {e}", file=sys.stderr)
                    last_tx = time.time()
                    cmd_idx += 1

                # Read whatever arrives
                try:
                    chunk = ser.read(96)
                except Exception as e:
                    print(f"[{ts()}] ❌ read error: {e}", file=sys.stderr)
                    break

                if chunk:
                    saw_any = True
                    if raw_dump:
                        sys.stderr.write(f"[{ts()}] RX << HEX:{hexdump(chunk)}  ASCII:{chunk.decode(errors='replace')}\n")

                    buf.extend(chunk)

                    # Parse CR/LF delimited lines
                    while True:
                        cut = None
                        for d in (b"\r\n", b"\n", b"\r"):
                            if d in buf:
                                line, _, rest = buf.partition(d)
                                buf = rest
                                cut = line
                                break
                        if cut is None:
                            break

                        txt = cut.decode(errors="ignore").strip()
                        if not txt:
                            continue
                        p = parse_line(txt)
                        if p:
                            N, raw_val, unit = p
                            if N is not None:
                                print(f"{N:.6f} N\t({raw_val} {unit})")
                            else:
                                print(f"{raw_val} {unit}")
                            if csv_path:
                                write_csv_row(csv_path, port, baud, N, raw_val, unit, txt)

                    # If there are no CR/LF, still try to parse inline snippets occasionally
                    if not any(x in buf for x in (b"\r", b"\n")) and len(buf) > 24:
                        tail = buf[-64:].decode(errors="ignore")
                        p = parse_line(tail)
                        if p:
                            N, raw_val, unit = p
                            if N is not None:
                                print(f"{N:.6f} N\t({raw_val} {unit}) [inline]")
                            else:
                                print(f"{raw_val} {unit} [inline]")
                            if csv_path:
                                write_csv_row(csv_path, port, baud, N, raw_val, unit, tail)

                time.sleep(0.002)

            if not saw_any:
                print(f"[{ts()}] ⏳ No bytes at {baud} within {seconds:.1f}s", file=sys.stderr)

    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped by user.")
        sys.exit(0)
    except Exception as e:
        print(f"[{ts()}] ERROR opening {port}@{baud}: {e}", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser(description="Probe + read HP-series force gauge")
    ap.add_argument("--port", default="", help="Serial port (default: auto-scan /dev/ttyUSB*;/dev/ttyACM*)")
    ap.add_argument("--bauds", default="9600,19200", help="Comma-separated baud list (default: 9600,19200)")
    ap.add_argument("--seconds", type=float, default=20.0, help="Probe seconds per baud (default 20)")
    ap.add_argument("--raw", action="store_true", help="Dump raw HEX+ASCII for each chunk to STDERR")
    ap.add_argument("--csv", default="", help="Optional CSV log path")
    ap.add_argument("--nudge-every", type=float, default=0.75, help="Seconds between handshake probes (0=off)")
    args = ap.parse_args()

    if args.csv:
        write_csv_header(args.csv)

    # Ports
    ports = [args.port] if args.port else find_ports(["/dev/ttyUSB*", "/dev/ttyACM*"])
    if not ports:
        print(f"[{ts()}] ⚠️  No serial ports found. Plug the gauge and check: ls -l /dev/ttyUSB* /dev/ttyACM*", file=sys.stderr)
        sys.exit(1)

    bauds = [int(b.strip()) for b in args.bauds.split(",") if b.strip()]

    print(f"[{ts()}] Ports: {ports}  Bauds: {bauds}", file=sys.stderr)
    for p in ports:
        for b in bauds:
            try_one(p, b, seconds=args.seconds, raw_dump=args.raw, csv_path=args.csv, nudge_every=args.nudge_every)

if __name__ == "__main__":
    main()
