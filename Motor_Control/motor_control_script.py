import time
from roboclaw import RoboClaw

# Update this with the appropriate serial port identified earlier
ROBOCLAW_PORT = '/dev/ttyACM0' # Mac: '/dev/tty.usbmodem14301'
BAUD_RATE = 38400

# Create RoboClaw object
roboclaw = Roboclaw(ROBOCLAW_PORT, BAUD_RATE)

# Open the serial port
if roboclaw.Open() == 0:
    print("Error: Could not open serial port")
    exit(1)

def main():
    address = 0x80  # Default address for RoboClaw

    # Read firmware version
    version = roboclaw.ReadVersion(address)
    if version[0] == False:
        print("Failed to read version")
    else:
        print("Version:", repr(version[1]))

    # Drive motor 1 forward at full speed
    roboclaw.ForwardM1(address, 95)
    print("Motor 1 forward at full speed")

    # Let motors run for 2 seconds
    time.sleep(20)

    # Stop motors
    roboclaw.ForwardM1(address, 0)
    roboclaw.ForwardM2(address, 0)
    print("Motors stopped")

if __name__ == "__main__":
    main()