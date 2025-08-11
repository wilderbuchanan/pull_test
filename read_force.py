#!/usr/bin/env python3
import re, sys, time, glob, argparse, binascii
import serial

unit_map = {"N": 1.0, "KN": 1000.0, "Kgf": 9.80665, "kgf": 9.80665,
            "lbf": 4.4482216153, "ozf": 0.278013850953}

def ts():
    return time.strftime("%H:%M:%S")

def hexdump(b):
    return binascii.hexlify(b).decode()

def parse_line(line):
    # Examples often look like: "+012.3 N", " -5.67 lbf", "123.4Kgf"
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", line)
    if not m: 
        return None
    val, unit = m.group(1), m.group(2)
    try:
        v = float(val)
    except:
        return None
    unit = unit.strip()
    factor = unit_map.get(unit)
    return {"raw": v, "unit": unit, "N": (v * factor) if factor else None}

def try_port(port, baud, per_try_seconds=10, send_every=1.0, nudges=(b"\r", b"\r\n", b"?\r")):
    print(f"[{ts()}] 🔌 Opening {port} @ {baud}", file=sys.stderr)
    with serial.Serial(
        port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.15,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        write_timeout=0.5,
    ) as ser:
        # Some devices start sending after DTR/RTS dance
        ser.setDTR(True);  ser.setRTS(True)
        time.sleep(0.05)
        ser.setDTR(False); ser.setRTS(False)
        time.sleep(0.05)
        ser.setDTR(True);  ser.setRTS(True)

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        t0 = time.time()
        last_send = 0
        linebuf = bytearray()
        saw_any_bytes = False

        while time.time() - t0 < per_try_seconds:
            # Periodically nudge to request output (acts like hitting PRINT)
            if send_every and (time.time() - last_send) >= send_every:
                for n in nudges:
                    try:
                        ser.write(n)
                    except Exception as e:
                        print(f"[{ts()}] ✉️  write error: {e}", file=sys.stderr)
                last_send = time.time()

            try:
                b = ser.read(1)
            except Exception as e:
                print(f"[{ts()}] ❌ read error: {e}", file=sys.stderr)
                break

            if b:
                if not saw_any_bytes:
                    print(f"[{ts()}] 📡 Receiving bytes…", file=sys.stderr)
                    saw_any_bytes = True

                # Build lines on \n or \r; also show raw bytes when weird
                if b in (b"\n", b"\r"):
                    if linebuf:
                        raw_line_bytes = bytes(linebuf)
                        try:
                            text = raw_line_bytes.decode(errors="ignore").strip()
                        except Exception:
                            text = ""
                        # Debug: show raw and hex
                        print(f"[{ts()}] RAW: {repr(text)}   HEX:{hexdump(raw_line_bytes)}", file=sys.stderr)

                        parsed = parse_line(text)
                        if parsed:
                            # Print a clean, machine-friendly line to STDOUT
                            if parsed["N"] is not None:
                                print(f"{parsed['N']:.6f} N\t({parsed['raw']} {parsed['unit']})")
                            else:
                                print(f"{parsed['raw']} {parsed['unit']}")
                            # Keep going; don’t exit so we can see more
                        linebuf.clear()
                else:
                    linebuf.extend(b)

            # Small sleep to avoid pegging CPU
            time.sleep(0.002)

        if not saw_any_bytes:
            print(f"[{ts()}] ⏳ No bytes received from {port}@{baud} in {per_try_seconds}s", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser(description="Serial force-gauge reader (debuggy).")
    ap.add_argument("--ports", default="/dev/ttyUSB*;/dev/ttyACM*",
                    help="Semicolon-separated globs to scan (default: /dev/ttyUSB*;/dev/ttyACM*)")
    ap.add_argument("--bauds", default="9600,19200,115200,4800,2400",
                    help="Comma-separated baud list to try")
    ap.add_argument("--per-try-seconds", type=float, default=12.0,
                    help="Seconds to wait per port/baud combo")
    ap.add_argument("--send-every", type=float, default=1.0,
                    help="Seconds between nudges (0=disable)")
    ap.add_argument("--no-nudge", action="store_true", help="Disable nudges")
    args = ap.parse_args()

    globs = [g.strip() for g in args.ports.split(";") if g.strip()]
    ports = []
    for g in globs:
        ports.extend(glob.glob(g))
    ports = sorted(set(ports))
    bauds = [int(b) for b in args.bauds.split(",") if b.strip()]

    if not ports:
        print(f"[{ts()}] ⚠️  No serial ports matched {globs}. Is the gauge plugged in? Permissions ok?", file=sys.stderr)
    else:
        print(f"[{ts()}] 🔎 Ports: {ports}", file=sys.stderr)

    print(f"[{ts()}] 🕒 Trying per combo for {args.per_try_seconds:.1f}s", file=sys.stderr)

    nudges = () if args.no_nudge else (b"\r", b"\r\n", b"?\r", b"Q\r")
    any_tried = False
    for p in ports:
        for b in bauds:
            any_tried = True
            try:
                try_port(p, b, args.per_try_seconds, args.send_every, nudges)
            except KeyboardInterrupt:
                print(f"\n[{ts()}] Interrup
