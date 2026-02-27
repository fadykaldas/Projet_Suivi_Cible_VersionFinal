import serial.tools.list_ports

print("Available COM ports:")
print("-" * 50)
ports = serial.tools.list_ports.comports()
if ports:
    for port in ports:
        print(f"  {port.device}: {port.description}")
else:
    print("  No COM ports found!")
print("-" * 50)

# Try to detect Arduino
arduino_ports = [p.device for p in serial.tools.list_ports.comports() if 'Arduino' in p.description]
if arduino_ports:
    print(f"\n✓ Arduino detected on: {arduino_ports[0]}")
else:
    print("\n✗ Arduino not detected")
