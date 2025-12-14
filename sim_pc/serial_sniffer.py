import serial
import time

PORT = "COM3"       # adjust if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
print(f"Sniffing {PORT} @ {BAUD}... Ctrl+C to stop.")

try:
    while True:
        data = ser.read(64)
        if data:
            print(f"{len(data):3d} bytes: {data.hex(' ')}")
        else:
            print("0 bytes (timeout)")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping.")
finally:
    ser.close()
