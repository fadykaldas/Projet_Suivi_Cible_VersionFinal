import serial
import time

PORT = "COM5"
BAUD = 115200

print("Waiting for device to be ready...")
time.sleep(3)

for i in range(5):
    try:
        print(f"Attempt {i+1}/5...")
        ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)
        print(f"✓ SUCCESS! Connected to {PORT}")
        
        # Test read
        print("Waiting for data from Arduino...")
        for j in range(5):
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"  Received: {line}")
        
        ser.close()
        break
    except Exception as e:
        print(f"  Failed: {e}")
        time.sleep(2)
