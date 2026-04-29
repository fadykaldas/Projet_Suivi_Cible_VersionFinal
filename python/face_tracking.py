import cv2
import serial
import time

# Configuration
PORT = "COM3"  # Change to your Arduino port
BAUD = 115200
CAM_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Smoothing
SMOOTHING_FACTOR = 0.1  # Lower = smoother but slower

def connect_serial(port):
    """Connect to Arduino serial."""
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        print(f"Connected to {port} at {BAUD} baud")
        return ser
    except Exception as e:
        print(f"Error connecting to {port}: {e}")
        return None

def map_value(value, from_min, from_max, to_min, to_max):
    """Map a value from one range to another."""
    if from_max == from_min:
        return to_min
    return int((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min)

def smooth_angle(current, target, factor):
    """Smooth angle transition."""
    return int(current + (target - current) * factor)

def main():
    # Load Haar cascade for face detection
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        print("ERROR: Failed to load Haar cascade")
        return

    # Connect to Arduino
    ser = connect_serial(PORT)
    if ser is None:
        return

    # Open camera
    cam = cv2.VideoCapture(CAM_INDEX)
    if not cam.isOpened():
        print("ERROR: Cannot open camera")
        return

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Initial servo positions
    current_angle_x = 90
    current_angle_y = 90

    print("Face tracking started. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("WARNING: Failed to read frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            if len(faces) > 0:
                # Take the largest face
                faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
                x, y, w, h = faces[0]

                # Calculate center of face
                center_x = x + w // 2
                center_y = y + h // 2

                # Map to servo angles (0-180)
                target_angle_x = map_value(center_x, 0, FRAME_WIDTH, 0, 180)
                target_angle_y = map_value(center_y, 0, FRAME_HEIGHT, 180, 0)

                # Smooth the angles
                current_angle_x = smooth_angle(current_angle_x, target_angle_x, SMOOTHING_FACTOR)
                current_angle_y = smooth_angle(current_angle_y, target_angle_y, SMOOTHING_FACTOR)

                # Send to Arduino
                command = f"{current_angle_x},{current_angle_y}\n"
                ser.write(command.encode())

                # Draw rectangle and center
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"X: {current_angle_x}, Y: {current_angle_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                # No face, smoothly return to center
                current_angle_x = smooth_angle(current_angle_x, 90, SMOOTHING_FACTOR)
                current_angle_y = smooth_angle(current_angle_y, 90, SMOOTHING_FACTOR)
                ser.write(f"{current_angle_x},{current_angle_y}\n".encode())
                cv2.putText(frame, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Face Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        cam.release()
        cv2.destroyAllWindows()
        if ser:
            ser.close()
        print("Done.")

if __name__ == "__main__":
    main()