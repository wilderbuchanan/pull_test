#!/usr/bin/env python3
"""
MXmoon/Lanetech HP-series force gauge (Modbus RTU @ 9600 8N1)
- Polls 01 03 0000 000D (13 holding registers)
- Decodes big-endian float at regs[0..1] (f00) as the live force (Newtons)
- Prints N (and optionally lbf/kgf), supports on-the-fly TARE ('z') and quit ('q')
- Optional CSV logging

Usage examples:
  python3 hp_force_stream.py
  python3 hp_force_stream.py --csv readings.csv --units N,lbf
  python3 hp_force_stream.py --hz 20
"""

import sys, time, struct, argparse, select, termios, tty
import serial

# ---------- Config ----------
REQ_SLAVE = 1
REQ_START = 0x0000
REQ_COUNT = 13            # 13 regs = 26 data bytes
POLL_HZ   = 10.0

# Which float holds the live reading? From your capture: f00 (regs 0..1).
LIVE_PAIR_INDEX = 0       # 0 -> regs[0..1], 2 -> regs[2..3], etc.

# Conversions
N_TO_LBF = 1.0 / 4.4482216152605
N_TO_KGF = 1.0 / 9.80665

# ---------- Helpers ----------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xA001
    return crc  # little-endian on the wire

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

# ---------- TTY (for 'z' tare / 'q' quit) ----------
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

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Stream force from HP-series gauge via Modbus RTU")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--hz", type=float, default=POLL_HZ, help="poll rate (Hz)")
    ap.add_argument("--units", default="N", help="comma list from {N,lbf,kgf}")
    ap.add_argument("--csv", default="", help="optional CSV path")
    ap.add_argument("--pair", type=int, default=LIVE_PAIR_INDEX, help="float pair index (0=regs0..1)")
    args = ap.parse_args()

    want_units = [u.strip().lower() for u in args.units.split(",") if u.strip()]
    header = ["time", "force_N"]
    if "lbf" in want_units: header.append("force_lbf")
    if "kgf" in want_units: header.append("force_kgf")

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write(",".join(header) + "\n")

    req = build_req(REQ_SLAVE, REQ_START, REQ_COUNT)
    print(f"[{ts()}] Opening {args.port} @ 9600 8N1; polling {args.hz:.1f} Hz")
    print(f"[{ts()}] Request: {req.hex(' ')} (slave=1 func=0x03 start=0x0000 count=13)")
    tare = 0.0

    try:
        with serial.Serial(args.port, 9600, timeout=0.12,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE) as ser:
            # common line wiggle like the PC app
            ser.setDTR(True); ser.setRTS(True); time.sleep(0.05)
            ser.reset_input_buffer(); ser.reset_output_buffer()

            kg = KeyGetter()
            period = 1.0 / max(args.hz, 0.5)
            print("[keys] 'z' = tare (zero), 'q' = quit")

            while True:
                t0 = time.time()
                # send request
                ser.write(req)

                # read header
                hdr = read_exact(ser, 3, timeout_s=0.35)
                if len(hdr) != 3 or hdr[0] != 1 or hdr[1] != 0x03:
                    # soft skip on timeout or error
                    time.sleep(max(0, period - (time.time()-t0)))
                    # check keys anyway
                    k = kg.get()
                    if k in ("q","Q"): break
                    if k in ("z","Z"): tare = 0.0  # can't tare without value, but reset offset
                    continue

                n = hdr[2]
                body = read_exact(ser, n + 2, timeout_s=0.45)  # data + CRC
                if len(body) != n + 2:
                    time.sleep(max(0, period - (time.time()-t0)))
                    k = kg.get()
                    if k in ("q","Q"): break
                    if k in ("z","Z"): tare = 0.0
                    continue

                # CRC check
                pkt_wo_crc = hdr + body[:-2]
                rx_crc = body[-2] | (body[-1] << 8)
                calc = crc16_modbus(pkt_wo_crc)
                if rx_crc != calc:
                    # bad frame; skip
                    time.sleep(max(0, period - (time.time()-t0)))
                    k = kg.get()
                    if k in ("q","Q"): break
                    if k in ("z","Z"): tare = 0.0
                    continue

                # parse regs
                data = body[:-2]
                regs = [ (data[i]<<8) | data[i+1] for i in range(0, len(data), 2) ]

                # decode chosen float (big-endian, two regs)
                i = max(0, min(args.pair, (len(regs)//2)-1))
                be = struct.pack(">HH", regs[2*i], regs[2*i + 1])
                force_N = struct.unpack(">f", be)[0] - tare

                # compute requested units
                out = [time.strftime("%H:%M:%S"), f"{force_N:.6f}"]
                if "lbf" in want_units: out.append(f"{force_N * N_TO_LBF:.6f}")
                if "kgf" in want_units: out.append(f"{force_N * N_TO_KGF:.6f}")

                print("  ".join(out))

                if args.csv:
                    with open(args.csv, "a", encoding="utf-8") as f:
                        f.write(",".join(out) + "\n")

                # hotkeys
                k = kg.get()
                if k in ("q","Q"):
                    break
                if k in ("z","Z"):
                    tare = force_N + tare  # set current reading to zero
                    print(f"[{ts()}] TARE applied; offset now {tare:.6f} N")

                # pace loop
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
        print(f"[{ts()}] Bye.")

if __name__ == "__main__":
    main()
