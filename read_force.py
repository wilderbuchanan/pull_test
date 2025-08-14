#!/usr/bin/env python3
import sys, time, struct, argparse, serial

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            m = crc & 1
            crc >>= 1
            if m:
                crc ^= 0xA001
    return crc  # little-endian on the wire

def build_req(slave=1, start=0x0000, count=0x000D) -> bytes:
    pdu = bytes([slave, 0x03, (start>>8)&0xFF, start&0xFF, (count>>8)&0xFF, count&0xFF])
    crc = crc16_modbus(pdu)
    return pdu + bytes([crc & 0xFF, (crc>>8) & 0xFF])

def read_exact(ser: serial.Serial, n: int, timeout_s: float=0.5) -> bytes:
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n and (time.time() - t0) < timeout_s:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)

def main():
    ap = argparse.ArgumentParser(description="Poll HP/MXmoon gauge via Modbus RTU @9600 and decode.")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--start", type=lambda x: int(x,0), default=0x0000)
    ap.add_argument("--count", type=int, default=13)
    ap.add_argument("--hz", type=float, default=10.0, help="poll rate (Hz)")
    args = ap.parse_args()

    req = build_req(args.slave, args.start, args.count)
    print(f"[{time.strftime('%H:%M:%S')}] Using request: {req.hex(' ')}  "
          f"(slave={args.slave}, func=0x03, start=0x{args.start:04X}, count={args.count})")

    try:
        with serial.Serial(args.port, 9600, timeout=0.1,
                           bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE) as ser:
            # Clean line states like the PC app
            ser.setDTR(True); ser.setRTS(True); time.sleep(0.05)
            ser.reset_input_buffer(); ser.reset_output_buffer()
            period = 1.0 / max(args.hz, 0.5)

            while True:
                t0 = time.time()
                ser.write(req)
                # Response header: addr(1) func(1) bytecount(1)
                hdr = read_exact(ser, 3, timeout_s=0.4)
                if len(hdr) != 3:
                    print("! Timeout waiting for header")
                elif hdr[0] != args.slave or hdr[1] != 0x03:
                    print(f"! Bad header: {hdr.hex(' ')}")
                else:
                    n = hdr[2]
                    body = read_exact(ser, n + 2, timeout_s=0.5)  # data + CRC
                    if len(body) == n + 2:
                        data = hdr + body  # full frame
                        # CRC check
                        pkt_wo_crc = data[:-2]
                        rx_crc = data[-2] | (data[-1] << 8)
                        calc = crc16_modbus(pkt_wo_crc)
                        if rx_crc != calc:
                            print(f"! CRC mismatch: rx=0x{rx_crc:04X} calc=0x{calc:04X}")
                        else:
                            # parse 16-bit registers
                            regs = [ (body[i]<<8) | body[i+1] for i in range(0, n, 2) ]
                            ts = time.strftime("%H:%M:%S")
                            reg_str = " ".join(f"{r:04X}" for r in regs)
                            print(f"[{ts}] regs[{args.start:#06x}..]: {reg_str}")

                            # try big-endian IEEE754 floats on even boundaries
                            floats = []
                            for i in range(0, len(regs)-1, 2):
                                be_bytes = struct.pack(">HH", regs[i], regs[i+1])
                                f = struct.unpack(">f", be_bytes)[0]
                                floats.append((i, f))
                            # show the interesting ones (non-zero)
                            nz = [f for f in floats if abs(f[1]) > 1e-9]
                            if nz:
                                msg = ", ".join([f"f{i:02d}={val:.6g}" for (i,val) in nz])
                                print(f"         floats(be, regs i..i+1): {msg}")
                    else:
                        print("! Incomplete payload")

                # keep a steady poll rate
                dt = time.time() - t0
                if dt < period: time.sleep(period - dt)

    except KeyboardInterrupt:
        print("\nBye.")
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(1)

if __name__ == "__main__":
    main()
