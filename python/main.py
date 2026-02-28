import cv2
import time
from collections import deque
import statistics
import serial
import serial.tools.list_ports
import argparse

# -------- CONFIG --------
PORT = "COM5"
BAUD = 115200

ON_THRESHOLD_CM = 60.0       # ouvrir caméra si distance <= 60 cm
OFF_THRESHOLD_CM = 70.0      # fermer caméra si distance >= 70 cm (hysteresis)
TRIGGER_CONFIRM_SEC = 2.0    # doit rester sous le seuil ON pendant 2s avant d'ouvrir

HOLD_SECONDS = 2.0           # visage détecté 2 sec => CIBLE DETECTEE
CAM_CLOSE_AFTER_RADAR_LOSS = 10.0  # close camera after 10s with no radar confirmation
CAM_INDEX = 0
# ------------------------


def open_camera():
    """Open camera, auto-selecting a working index if needed.

    Returns an opened `cv2.VideoCapture` or `None`.
    """
    indices = [CAM_INDEX] + [i for i in range(0, 5) if i != CAM_INDEX]
    for idx in indices:
        cam = cv2.VideoCapture(idx)
        if cam.isOpened():
            print(f"Camera opened on index {idx}")
            return cam
        try:
            cam.release()
        except Exception:
            pass
    print("ERROR: Camera not accessible on indices 0-4")
    return None


def parse_distance(line: str):
    """Parse distance from serial data with validation."""
    if not line or not line.strip():
        return None
    try:
        dist = float(line.strip())
        if 0 < dist < 500:   # valeurs raisonnables
            return dist
    except (ValueError, TypeError):
        pass
    return None
def select_serial_port(preferred_port=None):
    """Detect available COM ports and allow user to select one.
    If preferred_port is given and available, it will be returned.
    Returns port string like 'COM5' or None if none selected.
    """
    ports = list(serial.tools.list_ports.comports())
    if preferred_port:
        for p in ports:
            if p.device == preferred_port:
                return preferred_port

    arduino_ports = [p for p in ports if 'Arduino' in p.description or 'USB' in p.description]
    if len(arduino_ports) == 1:
        return arduino_ports[0].device

    if ports:
        print("Available COM ports:")
        for i, p in enumerate(ports, 1):
            print(f"  {i}. {p.device}: {p.description}")
        print("  0. Enter custom port")
        try:
            choice = input("Select port number (or 0): ").strip()
        except Exception:
            return None
        if choice == '0':
            custom = input("Enter port name (e.g. COM5): ").strip()
            return custom
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
        except Exception:
            return None

    return None


def connect_serial(preferred_port=None):
    """Connect to Arduino serial with retries and interactive fallback."""
    selected = preferred_port
    ports = list(serial.tools.list_ports.comports())
    if preferred_port not in [p.device for p in ports]:
        # let user select
        selected = select_serial_port(preferred_port)

    if not selected:
        raise RuntimeError("No serial port selected")

    max_retries = 4
    retry_delay = 2
    for attempt in range(max_retries):
        try:
            ser = serial.Serial(selected, BAUD, timeout=0.2)
            print(f"Connected to {selected} at {BAUD} baud")
            return ser
        except PermissionError as pe:
            print(f"Permission error opening {selected}: {pe}")
            if attempt < max_retries - 1:
                print(f"Retrying in {retry_delay}s... (Attempt {attempt+1}/{max_retries})")
                time.sleep(retry_delay)
            else:
                raise
        except Exception as e:
            print(f"Error opening {selected}: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
            else:
                print("Available COM ports:")
                for p in serial.tools.list_ports.comports():
                    print(f"  - {p.device}: {p.description}")
                raise



def main(simulate=False):
    # Haar cascade intégré OpenCV (pas besoin du xml dans ton dossier)
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        print("ERROR: Failed to load Haar cascade")
        return

    # Serial Arduino (prefer configured PORT)
    ser = None
    if simulate:
        class FakeSerial:
            def __init__(self, timeline):
                self.start = time.time()
                self.timeline = timeline

            def readline(self):
                t = time.time() - self.start
                for (s, e, v) in self.timeline:
                    if s <= t < e:
                        return (f"{v:.2f}\n").encode()
                return ("200.00\n").encode()

        # timeline: (start, end, distance_cm)
        timeline = [ (0.0, 2.0, 200.0), (2.0, 6.0, 50.0), (6.0, 120.0, 200.0) ]
        ser = FakeSerial(timeline)
        print("SIMULATION: using fake serial")
    else:
        try:
            ser = connect_serial(PORT)
        except Exception as e:
            print(f"Warning: serial unavailable: {e}")
            ser = None

    cam = None
    last_distance = None
    # keep short history and use median to filter noisy readings
    dist_history = deque(maxlen=5)

    # If serial unavailable, continue in camera-only mode
    if ser is None:
        print("Proceeding in camera-only mode (no radar).")
        # Open camera immediately for camera-only operation
        cam = open_camera()

    time.sleep(2)  # laisse Arduino reset

    # radar state machine
    detect_start = None    # moment distance went under ON_THRESHOLD_CM (arming)
    loss_start = None      # moment distance went above ON_THRESHOLD_CM while camera open
    target_start = None    # moment where face was first detected (for HOLD_SECONDS)
    state = 'idle'         # 'idle' or 'open'

    print("Running... Press 'q' to quit.")

    try:
        while True:
            # 1) Lire distance Arduino (si disponible)
            if ser is not None:
                try:
                    raw = ser.readline()
                    try:
                        line = raw.decode(errors="replace").strip()
                    except Exception:
                        line = str(raw)
                    dist = parse_distance(line)
                    # push to history (None when invalid)
                    dist_history.append(dist)
                    # compute median of valid values
                    valid = [d for d in dist_history if d is not None]
                    smooth = statistics.median(valid) if valid else None
                    # Debug: show raw, parsed and smoothed values
                    print(f"[SERIAL] raw={line!r} parsed={dist} smooth={smooth}")
                    if smooth is not None:
                        last_distance = smooth
                except Exception as e:
                    print(f"Serial read error: {e}")

            now = time.time()

            # 2) Radar state machine: require continuous detection for TRIGGER_CONFIRM_SEC to open,
            #    and close camera after CAM_CLOSE_AFTER_RADAR_LOSS seconds of no detection.
            detected_now = (last_distance is not None) and (last_distance <= ON_THRESHOLD_CM)

            if state == 'idle':
                if detected_now:
                    if detect_start is None:
                        detect_start = now
                    elif (now - detect_start) >= TRIGGER_CONFIRM_SEC:
                        # confirmed -> open camera
                        print(f"Radar confirmed (stable {TRIGGER_CONFIRM_SEC}s) -> opening camera")
                        cam = open_camera()
                        if cam is not None:
                            state = 'open'
                            target_start = None
                        detect_start = None
                        loss_start = None
                else:
                    # reset arming when not detecting
                    detect_start = None

            elif state == 'open':
                if not detected_now:
                    if loss_start is None:
                        loss_start = now
                    elif (now - loss_start) >= CAM_CLOSE_AFTER_RADAR_LOSS:
                        print(f"Radar lost for {CAM_CLOSE_AFTER_RADAR_LOSS}s -> closing camera")
                        if cam is not None:
                            cam.release()
                            cam = None
                            cv2.destroyAllWindows()
                        state = 'idle'
                        target_start = None
                        detect_start = None
                        loss_start = None
                else:
                    # still detecting -> reset loss timer
                    loss_start = None

            # debug state
            print(f"[RADAR] state={state} last_distance={last_distance} detect_start={detect_start} loss_start={loss_start}")

            # 5) Si caméra active: détection visage + timer 2 secondes
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

                detected = len(faces) > 0  # ici: "objet" = visage (test)

                # Dessiner rectangles
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Timer "stable 2 sec"
                if detected:
                    if target_start is None:
                        target_start = now
                    elapsed = now - target_start
                else:
                    target_start = None
                    elapsed = 0.0

                # UI: distance
                dist_text = f"Distance: {last_distance:.1f} cm" if last_distance is not None else "Distance: N/A"
                cv2.putText(frame, dist_text, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

                # UI: statut radar (use state machine)
                if state == 'open':
                    cv2.putText(frame, "RADAR CONFIRME", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                elif detect_start is not None:
                    cv2.putText(frame, "DETECTION... (confirmation)", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "RADAR: N/A", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)

                # UI: cible détectée si stable
                if detected and elapsed >= HOLD_SECONDS:
                    cv2.putText(frame, "CIBLE DETECTEE", (20, 140),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                else:
                    cv2.putText(frame, f"Face hold: {elapsed:.1f}s / {HOLD_SECONDS:.1f}s", (20, 140),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

                cv2.imshow("Camera (OpenCV)", frame)

                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            # Optionnel: debug console si caméra fermée
            # else:
            #     if last_distance is not None:
            #         print(f"Distance: {last_distance:.1f} cm")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        print("Cleaning up...")
        try:
            if cam is not None:
                cam.release()
        except:
            pass

        try:
            cv2.destroyAllWindows()
        except:
            pass

        try:
            ser.close()
        except:
            pass

        print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Radar -> camera controller")
    parser.add_argument("--simulate", action="store_true", help="Run with fake serial data for testing")
    args = parser.parse_args()
    main(simulate=args.simulate)