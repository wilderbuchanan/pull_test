#!/usr/bin/env python3
import argparse, sys, time, glob, re, binascii, os
import serial

UNIT_MAP = {
    "N": 1.0, "kN": 1000.0, "KN": 1000.0,
    "kgf": 9.80665, "Kgf": 9.80665, "gf": 0.00980665,
    "lbf": 4.4482216152605, "ozf": 0.278013850953
}

# Commands many HP/MXmoon firmwares react to (poll/print/version/zero)
ASCII_POKES = [b"\r", b"\n", b"ONLINE\r", b"OnLine\r", b"Online\r",
               b"READ\r", b"PRINT\r", b"V\r", b"Z\r", b"?\r", b"R\r", b"P\r", b"S\r", b"D\r"]
CTRL_POKES  = [b"\x05", b"\x04", b"\x11", b"\x13", b"\x02", b"\x03"]  # ENQ,EOT,XON,XOFF,STX,ETX

def ts(): return time.strftime("%H:%M:%S")
def hexdump(b: bytes) -> str: return binascii.hexlify(b).decode()

def find_ports():
    ports = sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")))
    return ports

def parse_line(text: str):
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m:
        return None
    try:
        raw = float(m.group(1))
    except ValueError:
        return None
    unit = m.group(2).strip()
    factor = UNIT_MAP.get(unit)
    N = (raw * factor) if factor else None
    return N, raw, unit

def try_one(port, baud, seconds=22.0, raw_dump=True, nudge_every=0.75):
    print(f"[{ts()}] 🔌 Open {port} @ {baud}")
    try:
        with serial.Serial(
            port=port, baudrate=baud, timeout=0.18,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            xonxoff=False, rtscts=False, dsrdtr=False, write_timeout=0.5
        ) as ser:
            # Wake lines the way vendor apps often do
            ser.setDTR(True);  ser.setRTS(False); time.sleep(0.05)
            ser.setDTR(False); ser.setRTS(True);  time.sleep(0.05)
            ser.setDTR(True);  ser.setRTS(True)
            ser.reset_input_buffer(); ser.reset_output_buffer()

            print(f"[{ts()}] 👂 Listening; probing for up to {seconds:.1f}s (press SEND on the gauge once)")
            t0 = time.time()
            last_tx = 0.0
            idx_ascii = 0
            idx_ctrl  = 0
            buf = bytearray()
            saw_any = False

            while time.time() - t0 < seconds:
                # periodic ASCII & control pokes
                if nudge_every > 0 and (time.time() - last_tx) >= nudge_every:
                    # alternate: ASCII, CTRL, ASCII, CTRL...
                    if ((idx_ascii + idx_ctrl) % 2) == 0:
                        cmd = ASCII_POKES[idx_ascii % len(ASCII_POKES)]
                        idx_ascii += 1
                    else:
                        cmd = CTRL_POKES[idx_ctrl % len(CTRL_POKES)]
                        idx_ctrl += 1
                    try:
                        ser.write(cmd)
                        print(f"[{ts()}] TX >> {repr(cmd)}", file=sys.stderr)
                    except Exception as e:
                        print(f"[{ts()}] ❌ write error: {e}", file=sys.stderr)
                    last_tx = time.time()

                # read anything available
                chunk = ser.read(96)
                if chunk:
                    saw_any = True
                    if raw_dump:
                        sys.stderr.write(f"[{ts()}] RX << HEX:{hexdump(chunk)}  ASCII:{chunk.decode(errors='replace')}\n")
                    buf.extend(chunk)

                    # parse CR/LF delimited lines
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
                            N, raw_v, unit = p
                            if N is not None:
                                print(f"{N:.6f} N\t({raw_v} {unit})")
                            else:
                                print(f"{raw_v} {unit}")

                    # if device streams no terminators, try inline parse on tail
                    if not any(x in buf for x in (b"\r", b"\n")) and len(buf) > 24:
                        tail = buf[-64:].decode(errors="ignore")
                        p = parse_line(tail)
                        if p:
                            N, raw_v, unit = p
                            if N is not None:
                                print(f"{N:.6f} N\t({raw_v} {unit}) [inline]")
                            else:
                                print(f"{raw_v} {unit} [inline]")

                time.sleep(0.002)

            if not saw_any:
                print(f"[{ts()}] ⏳ No bytes at {baud} within {seconds:.1f}s", file=sys.stderr)

    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped by user."); sys.exit(0)
    except Exception as e:
        print(f"[{ts()}] ERROR opening {port}@{baud}: {e}", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser(description="Probe + read HP-series force gauge (CH341 USB-Serial)")
    ap.add_argument("--port", default="", help="Serial port (default: auto /dev/ttyUSB*;/dev/ttyACM*)")
    ap.add_argument("--bauds", default="9600,19200", help="Comma-separated baud list")
    ap.add_argument("--seconds", type=float, default=22.0, help="Seconds per baud")
    ap.add_argument("--raw", action="store_true", help="Dump raw HEX+ASCII to STDERR")
    args = ap.parse_args()

    ports = [args.port] if args.port else find_ports()
    if not ports:
        print(f"[{ts()}] ⚠️  No serial ports found. Plug the gauge, then check: ls -l /dev/ttyUSB* /dev/ttyACM*", file=sys.stderr)
        sys.exit(1)

    bauds = [int(b.strip()) for b in args.bauds.split(",") if b.strip()]
    print(f"[{ts()}] Ports: {ports}  Bauds: {bauds}", file=sys.stderr)
    for p in ports:
        for b in bauds:
            try_one(p, b, seconds=args.seconds, raw_dump=True)

if __name__ == "__main__":
    main()
