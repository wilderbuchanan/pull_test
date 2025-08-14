#!/usr/bin/env python3
import sys, time, glob, re, binascii, argparse
import serial

BAUDS = [9600, 19200, 4800, 38400, 57600, 115200]
FRAMES = [
    ("8N1", serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE),
    ("7E1", serial.SEVENBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE),
    ("8E1", serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE),
    ("8O1", serial.EIGHTBITS, serial.PARITY_ODD,  serial.STOPBITS_ONE),
]
FLOWS = [
    ("no-flow",  False, False, False),
    ("rtscts",   False, True,  False),
    ("dsrdtr",   False, False, True),
    ("xonxoff",  True,  False, False),
]
ASCII_POKES = [b"\r", b"\n", b"READ\r", b"PRINT\r", b"ONLINE\r", b"V\r", b"?\r", b"R\r", b"P\r", b"S\r", b"D\r"]
CTRL_POKES  = [b"\x05", b"\x04", b"\x11", b"\x13", b"\x02", b"\x03"]  # ENQ,EOT,XON,XOFF,STX,ETX
UNIT_MAP = {"N":1.0, "kN":1000.0, "KN":1000.0, "kgf":9.80665, "Kgf":9.80665, "lbf":4.4482216152605, "ozf":0.278013850953}

def ts(): return time.strftime("%H:%M:%S")
def hexdump(b: bytes) -> str: return binascii.hexlify(b).decode()
def parse_line(text: str):
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m: return None
    try: raw = float(m.group(1))
    except ValueError: return None
    unit = m.group(2).strip()
    factor = UNIT_MAP.get(unit)
    return ((raw*factor if factor else None), raw, unit)

def open_and_probe(port, baud, frame, flow, seconds=16.0, raw_dump=True):
    fname, bytesize, parity, stopbits = frame
    flowname, xonxoff, rtscts, dsrdtr = flow
    print(f"[{ts()}] 🔌 {port} @ {baud} {fname} {flowname}")
    try:
        with serial.Serial(
            port=port, baudrate=baud, timeout=0.18,
            bytesize=bytesize, parity=parity, stopbits=stopbits,
            xonxoff=xonxoff, rtscts=rtscts, dsrdtr=dsrdtr, write_timeout=0.5
        ) as ser:
            # Wake lines
            ser.setDTR(True); ser.setRTS(False); time.sleep(0.05)
            ser.setDTR(False); ser.setRTS(True);  time.sleep(0.05)
            ser.setDTR(True); ser.setRTS(True);   time.sleep(0.05)
            # Pulse BREAK briefly
            try:
                ser.send_break(0.2)
            except Exception:
                pass
            ser.reset_input_buffer(); ser.reset_output_buffer()

            print(f"[{ts()}] 👂 Listening… (press SEND on gauge once)")
            t0 = time.time()
            last_tx = 0.0
            idx_a = 0; idx_c = 0
            buf = bytearray()
            saw = False

            while time.time() - t0 < seconds:
                # alternate ASCII/CTRL pokes
                if time.time() - last_tx >= 0.7:
                    pkt = ASCII_POKES[idx_a % len(ASCII_POKES)] if ((idx_a + idx_c) % 2 == 0) else CTRL_POKES[idx_c % len(CTRL_POKES)]
                    ser.write(pkt); last_tx = time.time()
                    if (idx_a + idx_c) % 2 == 0: idx_a += 1
                    else: idx_c += 1
                    sys.stderr.write(f"[{ts()}] TX >> {repr(pkt)}\n")

                # read
                b = ser.read(128)
                if b:
                    saw = True
                    if raw_dump:
                        sys.stderr.write(f"[{ts()}] RX << HEX:{hexdump(b)}  ASCII:{b.decode(errors='replace')}\n")
                    buf.extend(b)
                    # CR/LF parsing
                    while True:
                        cut = None
                        for d in (b"\r\n", b"\n", b"\r"):
                            if d in buf:
                                line, _, rest = buf.partition(d)
                                buf = rest
                                cut = line
                                break
                        if cut is None: break
                        txt = cut.decode(errors="ignore").strip()
                        if not txt: continue
                        p = parse_line(txt)
                        if p:
                            N, raw_val, unit = p
                            if N is not None: print(f"{N:.6f} N\t({raw_val} {unit})")
                            else:            print(f"{raw_val} {unit}")

                    # inline parse if no terminators
                    if not any(x in buf for x in (b"\r", b"\n")) and len(buf) > 24:
                        tail = buf[-64:].decode(errors="ignore")
                        p = parse_line(tail)
                        if p:
                            N, raw_val, unit = p
                            if N is not None: print(f"{N:.6f} N\t({raw_val} {unit}) [inline]")
                            else:            print(f"{raw_val} {unit} [inline]")

                time.sleep(0.002)

            if not saw:
                print(f"[{ts()}] ⏳ No bytes at {baud} {fname} {flowname}", file=sys.stderr)

    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped."); sys.exit(0)
    except Exception as e:
        print(f"[{ts()}] ERROR opening {port}@{baud} {fname} {flowname}: {e}", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="e.g. /dev/ttyUSB0 (default: auto-scan)")
    ap.add_argument("--seconds", type=float, default=16.0)
    args = ap.parse_args()

    ports = [args.port] if args.port else sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")))
    if not ports:
        print(f"[{ts()}] ⚠️ No ports found. Try: ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id", file=sys.stderr); sys.exit(1)

    print(f"[{ts()}] Ports: {ports}")
    for p in ports:
        for b in BAUDS:
            for frame in FRAMES:
                for flow in FLOWS:
                    open_and_probe(p, b, frame, flow, seconds=args.seconds)

if __name__ == "__main__":
    main()
