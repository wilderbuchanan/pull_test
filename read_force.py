#!/usr/bin/env python3
import sys, time, glob, re, binascii, argparse
import serial

UNIT_MAP = {"N":1.0,"KN":1000.0,"Kgf":9.80665,"kgf":9.80665,"lbf":4.4482216153,"ozf":0.278013850953}

# Commands many low-cost force gauges respond to (poll current reading, print, etc.)
NUDGE_CMDS = [b"\r", b"\n", b"?\r", b"Q\r", b"R\r", b"P\r", b"S\r", b"D\r", b"READ\r", b"PRINT\r", b"V\r", b"Z\r"]

def ts():
    return time.strftime("%H:%M:%S")

def hexdump(b: bytes) -> str:
    return binascii.hexlify(b).decode()

def parse_line(text: str):
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m:
        return None
    val, unit = m.group(1), m.group(2)
    try:
        v = float(val)
    except ValueError:
        return None
    factor = UNIT_MAP.get(unit)
    return (v*factor if factor else None, v, unit)

def try_one(port, baud, seconds=18, write_every=0.75):
    print(f"[{ts()}] 🔌 Open {port} @ {baud}")
    try:
        with serial.Serial(
            port=port, baudrate=baud, timeout=0.15,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            xonxoff=False, rtscts=False, dsrdtr=False, write_timeout=0.5
        ) as ser:
            # Wake the device a bit
            ser.setDTR(True); ser.setRTS(True); time.sleep(0.05)
            ser.setDTR(False); ser.setRTS(False); time.sleep(0.05)
            ser.setDTR(True); ser.setRTS(True)
            ser.reset_input_buffer(); ser.reset_output_buffer()

            print(f"[{ts()}] 👂 Listening (Ctrl+C to stop) … probing for up to {seconds}s")
            t0 = time.time()
            last_tx = 0.0
            cmd_idx = 0
            buf = bytearray()
            saw_any = False

            while time.time() - t0 < seconds:
                # Periodically send a probe command
                if write_every and (time.time() - last_tx) >= write_every:
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
                    chunk = ser.read(64)
                except Exception as e:
                    print(f"[{ts()}] ❌ read error: {e}", file=sys.stderr)
                    break
                if chunk:
                    saw_any = True
                    # Raw view
                    sys.stderr.write(f"[{ts()}] RX << HEX:{hexdump(chunk)}  ASCII:{chunk.decode(errors='replace')}\n")
                    buf.extend(chunk)

                    # Parse CR/LF-delimited lines
                    while True:
                        cut = None
                        for d in (b"\r", b"\n"):
                            if d in buf:
                                line, _, rest = buf.partition(d)
                                buf = rest
                                cut = line
                                break
                        if cut is None:
                            break
                        txt = cut.decode(errors="ignore").strip()
                        if txt:
                            p = parse_line(txt)
                            if p:
                                N, raw, unit = p
                                if N is not None:
                                    print(f"{N:.6f} N\t({raw} {unit})")
                                else:
                                    print(f"{raw} {unit}")
                time.sleep(0.002)

            if not saw_any:
                print(f"[{ts()}] ⏳ No bytes at {baud} within {seconds}s", file=sys.stderr)

    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped by user.")
        sys.exit(0)
    except Exception as e:
        print(f"[{ts()}] ERROR opening {port}@{baud}: {e}", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser(description="Probe + read HP-series force gauge")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    ap.add_argument("--bauds", default="9600,19200", help="Comma-separated bauds to try")
    ap.add_argument("--seconds", type=float, default=18.0, help="Seconds per baud to probe")
    args = ap.parse_args()

    bauds = [int(b) for b in args.bauds.split(",") if b.strip()]
    print(f"[{ts()}] Ports: {args.port}  Bauds: {bauds}", file=sys.stderr)
    for b in bauds:
        try_one(args.port, b, seconds=args.seconds)

if __name__ == "__main__":
    main()
