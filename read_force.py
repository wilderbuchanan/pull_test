#!/usr/bin/env python3
import serial, binascii, time, re

PORT = "/dev/ttyUSB0"
BAUD = 9600
TIMEOUT = 0.2  # seconds

unit_map = {
    "N": 1.0, "KN": 1000.0,
    "Kgf": 9.80665, "kgf": 9.80665,
    "lbf": 4.4482216153, "ozf": 0.278013850953
}

def ts():
    return time.strftime("%H:%M:%S")

def parse_line(text):
    """Try to parse a line like '+012.3 N' or '123.4Kgf'"""
    m = re.search(r"([+-]?\d+(?:\.\d+)?)\s*([A-Za-z]+)", text)
    if not m:
        return None
    val_str, unit = m.group(1), m.group(2)
    try:
        v = float(val_str)
    except:
        return None
    factor = unit_map.get(unit)
    return {
        "raw": v,
        "unit": unit,
        "N": (v * factor) if factor else None
    }

def main():
    print(f"[{ts()}] Opening {PORT} @ {BAUD}")
    try:
        with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
            print(f"[{ts()}] Listening... Press CTRL+C to stop.")
            buffer = bytearray()
            while True:
                data = ser.read(64)  # read up to 64 bytes at a time
                if not data:
                    continue

                # Show raw data in hex and ASCII
                hex_str = binascii.hexlify(data).decode()
                ascii_str = data.decode(errors='replace')
                print(f"[{ts()}] HEX: {hex_str}  ASCII: {ascii_str}")

                # Accumulate into buffer to detect any \n/\r delimited lines
                buffer.extend(data)
                while b"\n" in buffer or b"\r" in buffer:
                    for delim in (b"\n", b"\r"):
                        if delim in buffer:
                            line, _, rest = buffer.partition(delim)
                            buffer = rest
                            try:
                                text_line = line.decode(errors='ignore').strip()
                            except:
                                text_line = ""
                            if text_line:
                                parsed = parse_line(text_line)
                                if parsed:
                                    if parsed["N"] is not None:
                                        print(f"[{ts()}] PARSED: {parsed['N']:.6f} N  ({parsed['raw']} {parsed['unit']})")
                                    else:
                                        print(f"[{ts()}] PARSED: {parsed['raw']} {parsed['unit']}")
    except KeyboardInterrupt:
        print(f"\n[{ts()}] Stopped by user.")
    except Exception as e:
        print(f"[{ts()}] ERROR: {e}")

if __name__ == "__main__":
    main()
