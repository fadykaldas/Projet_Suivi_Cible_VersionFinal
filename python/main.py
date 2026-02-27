import cv2
import time
import serial
import serial.tools.list_ports
import sys

# -------- CONFIG --------
PORT = "COM5"
BAUD = 115200

DIST_THRESHOLD_CM = 60.0   # si distance < 60cm => on active caméra
HOLD_SECONDS = 2.0         # si objet détecté 2 sec => CIBLE DETECTEE
CAM_INDEX = 0
# ------------------------

def open_camera():
    """Open camera with error handling"""
    try:
        cam = cv2.VideoCapture(CAM_INDEX)
        if not cam.isOpened():
            print("ERROR: Camera not accessible")
            return None
        return cam
    except Exception as e:
        print(f"ERROR opening camera: {e}")
        return None

def parse_distance(line):
    """Parse distance from serial data with validation"""
    if not line or not line.strip():
        return None
    try:
        dist = float(line.strip())
        # Validate distance is reasonable (positive, not too large)
        if 0 < dist < 500:
            return dist
    except (ValueError, TypeError):
        pass
    return None

def main():
    """Main target tracking loop"""
    # Haarcascade intégré à OpenCV (pas besoin du xml dans ton dossier)
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        print("ERROR: Failed to load Haar cascade")
        return

    # Serial Arduino - with retry logic
    ser = None
    max_retries = 3
    retry_delay = 2
    
    for attempt in range(max_retries):
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.2)
            print(f"✓ Connected to {PORT} at {BAUD} baud")
            break
        except PermissionError:
            if attempt < max_retries - 1:
                print(f"COM port locked. Retrying in {retry_delay}s... (Attempt {attempt+1}/{max_retries})")
                time.sleep(retry_delay)
            else:
                print(f"ERROR: Cannot access {PORT} after {max_retries} attempts")
                print("This usually means Arduino IDE or another app is using the port.")
                print("Close Arduino IDE and any other serial port applications, then retry.")
                return
        except Exception as e:
            if attempt < max_retries - 1:
                print(f"Error: {e}. Retrying... (Attempt {attempt+1}/{max_retries})")
                time.sleep(retry_delay)
            else:
                print(f"ERROR: Cannot open serial port {PORT}: {e}")
                print("Make sure Arduino is connected and COM port is correct.")
                print("Available COM ports:")
                try:
                    for port in serial.tools.list_ports.comports():
                        print(f"  - {port.device}: {port.description}")
                except:
                    pass
                return
    
    if ser is None:
        return
    
    time.sleep(2)  # laisse Arduino reset

    cam = None
    target_start = None
    last_distance = None

    print("Running... Press 'q' to quit.")

    try:
        while True:
            # Lire distance Arduino
            try:
                line = ser.readline().decode(errors="ignore").strip()
                dist = parse_distance(line)
                if dist is not None:
                    last_distance = dist
            except Exception as e:
                print(f"Serial read error: {e}")
                pass

            radar_trigger = (last_distance is not None) and (0 < last_distance < DIST_THRESHOLD_CM)

            # Ouvrir/fermer caméra selon radar
            if radar_trigger and cam is None:
                cam = open_camera()
                target_start = None

            if (not radar_trigger) and cam is not None:
                cam.release()
                cam = None
                cv2.destroyAllWindows()
                target_start = None

            # Si caméra active: détection
            if cam is not None:
                ret, frame = cam.read()
                if not ret:
                    print("WARNING: Failed to read frame from camera")
                    cam.release()
                    cam = None
                    cv2.destroyAllWindows()
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4)

                detected = len(faces) > 0  # pour l'instant: "objet" = visage (test simple)

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Timer 2 sec
                if detected:
                    if target_start is None:
                        target_start = time.time()
                    elapsed = time.time() - target_start
                else:
                    target_start = None
                    elapsed = 0

                # UI
                cv2.putText(frame, f"Distance: {last_distance:.1f} cm" if last_distance else "Distance: N/A",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

                if detected and elapsed >= HOLD_SECONDS:
                    cv2.putText(frame, "CIBLE DETECTEE", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                else:
                    cv2.putText(frame, "Radar actif - recherche...", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                cv2.imshow("Camera (OpenCV)", frame)

                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"ERROR in main loop: {e}")
    finally:
        # Cleanup - always execute
        print("Cleaning up...")
        if cam is not None:
            try:
                cam.release()
            except:
                pass
        cv2.destroyAllWindows()
        
        try:
            ser.close()
        except:
            pass
        
        print("Done.")

if __name__ == "__main__":
    main()
