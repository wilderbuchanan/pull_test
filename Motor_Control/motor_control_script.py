import time
from roboclaw_3 import Roboclaw

PORT = "/dev/ttyACM0"
BAUD = 38400
ADDR = 0x80  # default address

rc = Roboclaw(PORT, BAUD)

def _decode(maybe_bytes_or_str):
    try:
        return maybe_bytes_or_str.decode("utf-8", "ignore")
    except Exception:
        return str(maybe_bytes_or_str)

def _read_version():
    ok, ver = rc.ReadVersion(ADDR)
    if ok:
        print("Version:", _decode(ver))
    else:
        print("Failed to read version")

def _reset_encoders():
    # Some variants have ResetEncoders; others only SetEncM1/SetEncM2
    try:
        rc.ResetEncoders(ADDR)
        print("Encoders reset to 0")
    except AttributeError:
        # Fallback: set each encoder to 0 if available
        try:
            rc.SetEncM1(ADDR, 0)
            rc.SetEncM2(ADDR, 0)
            print("Encoders set to 0")
        except Exception:
            print("Skip encoder reset (method not available)")

def _read_enc_m1():
    """
    Returns (ok, count) in a normalized way.
    """
    out = rc.ReadEncM1(ADDR)
    # common forms: (ok, value), or (ok, value, status)
    if isinstance(out, tuple):
        if len(out) >= 2:
            return bool(out[0]), int(out[1])
    return False, 0

def _read_speed_m1():
    """
    Returns (ok, speed) if the library provides ReadSpeedM1.
    """
    try:
        out = rc.ReadSpeedM1(ADDR)
    except AttributeError:
        return False, 0
    if isinstance(out, tuple) and len(out) >= 2:
        return bool(out[0]), int(out[1])
    return False, 0

def main():
    if rc.Open() == 0:
        print("Error: Could not open serial port:", PORT)
        return

    _read_version()

    # If your encoder is wired (+5V, GND, EN1, EN2), this will give sane numbers
    _reset_encoders()

    # Start gently (0..127). 32~slow, 95~fast, 127=max.
    rc.ForwardM1(ADDR, 32)
    print("Motor 1 forward (speed cmd=32)")

    t_end = time.time() + 100.0
    while time.time() < t_end:
        ok_c, cnt = _read_enc_m1()
        ok_s, spd = _read_speed_m1()
        if ok_c and ok_s:
            print(f"Enc={cnt:8d}  Speed={spd:8d}")
        elif ok_c:
            print(f"Enc={cnt:8d}")
        else:
            print("Encoder read failed")
        time.sleep(0.2)

    # Stop
    rc.ForwardM1(ADDR, 0)
    rc.ForwardM2(ADDR, 0)
    print("Motors stopped")

if __name__ == "__main__":
    main()
