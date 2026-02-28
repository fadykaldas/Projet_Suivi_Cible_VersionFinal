import sys
import time
import re
import math
import statistics
from collections import deque

import cv2
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QMessageBox
)

# -------- CONFIG --------
BAUD = 115200
SERIAL_TIMEOUT_SEC = 0.05

# Radar -> camera
ON_THRESHOLD_CM = 60.0
OFF_THRESHOLD_CM = 70.0
TRIGGER_CONFIRM_SEC = 2.0
CAM_CLOSE_AFTER_RADAR_LOSS = 10.0

# Camera / OpenCV
CAM_INDEX = 0
HOLD_SECONDS = 2.0                 # si objet détecté >= 2s => CIBLE DETECTEE
DETECT_EVERY_N_FRAMES = 2          # 1 = chaque frame (plus lourd), 2 = 1 frame sur 2

# Radar rendu
RADAR_MAX_RANGE_CM = 200.0
POINT_TTL_SEC = 2.0
SWEEP_TRAIL = 18

# Calibration angle radar
SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0
RADAR_FLIP = True
RADAR_OFFSET_DEG = 0.0

SHOW_DEBUG = True
# ------------------------


def list_com_ports():
    ports = list(serial.tools.list_ports.comports())
    return [(p.device, p.description) for p in ports]


def frame_to_pixmap(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    bytes_per_line = ch * w
    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
    return QPixmap.fromImage(qimg)


class SerialParser:
    """
    Supporte:
    - "angle,distance"  ex: "161,8"
    - "Angle: 161 Distance: 8"
    - 2 lignes séparées: "161" puis "8"
    - distance seule (angle restera None)
    """
    def __init__(self):
        self.pending_angle = None

    def feed(self, line: str):
        if not line:
            return None, None

        s = line.strip()
        low = s.lower()
        nums = re.findall(r"\d+(?:\.\d+)?", s)
        if not nums:
            return None, None

        if len(nums) >= 2:
            a = float(nums[0])
            d = float(nums[-1])
            self.pending_angle = None
            return a, d

        n = float(nums[0])

        if ("ang" in low) or ("deg" in low):
            self.pending_angle = n
            return n, None

        if ("dist" in low) or ("cm" in low):
            if self.pending_angle is not None:
                a = self.pending_angle
                self.pending_angle = None
                return a, n
            return None, n

        # numeric-only lines: assume alternating angle then distance
        if self.pending_angle is None:
            self.pending_angle = n
            return None, None
        a = self.pending_angle
        self.pending_angle = None
        return a, n


class RadarWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(520, 380)

        self.max_range = float(RADAR_MAX_RANGE_CM)
        self.last_angle = 90.0
        self.last_distance = None

        self.sweep_angles = deque(maxlen=SWEEP_TRAIL)
        self.points = deque(maxlen=400)  # (angle, dist, timestamp)

    def map_angle(self, angle_deg: float) -> float:
        a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle_deg)))

        if SERVO_MAX_ANGLE != SERVO_MIN_ANGLE:
            a = (a - SERVO_MIN_ANGLE) * 180.0 / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)

        a = a + RADAR_OFFSET_DEG
        a = max(0.0, min(180.0, a))

        if RADAR_FLIP:
            a = 180.0 - a

        return a

    def update_data(self, angle_raw, dist_cm):
        now = time.time()

        if angle_raw is not None:
            ang = self.map_angle(angle_raw)
            self.last_angle = ang
            self.sweep_angles.appendleft(ang)
        else:
            self.sweep_angles.appendleft(self.last_angle)

        if dist_cm is not None:
            self.last_distance = float(dist_cm)
            if 0.0 < self.last_distance <= self.max_range:
                self.points.append((self.last_angle, self.last_distance, now))

        self.update()

    def paintEvent(self, event):
        now = time.time()
        while self.points and (now - self.points[0][2]) > POINT_TTL_SEC:
            self.points.popleft()

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(0, 0, 0))

        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.95
        radius = min(w * 0.48, h * 0.9)

        green = QColor(0, 255, 80)
        dark_green = QColor(0, 110, 40)

        p.setPen(QPen(dark_green, 2))
        for frac in [0.25, 0.5, 0.75, 1.0]:
            r = radius * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0 * 16, 180 * 16)

        for ang in [0, 30, 60, 90, 120, 150, 180]:
            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(int(cx), int(cy), int(x), int(y))

        p.setPen(QPen(green))
        p.setFont(QFont("Consolas", 10))
        p.drawText(10, 20, "Radar Display")

        for frac in [0.5, 1.0]:
            r = radius * frac
            cm = self.max_range * frac
            p.drawText(int(cx + 10), int(cy - r - 5), f"{cm:.0f} cm")

        angle_txt = f"Angle: {self.last_angle:.0f}°"
        dist_txt = f"Distance: {self.last_distance:.0f} cm" if self.last_distance is not None else "Distance: N/A"
        p.drawText(10, 40, angle_txt)
        p.drawText(10, 60, dist_txt)

        for i, ang in enumerate(self.sweep_angles):
            alpha = max(20, 255 - i * 12)
            pen = QPen(QColor(0, 255, 80, alpha), 4 if i == 0 else 2)
            p.setPen(pen)

            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        for (ang, dist, ts) in self.points:
            age = now - ts
            fade = max(40, int(255 * (1.0 - age / POINT_TTL_SEC)))
            p.setPen(QPen(QColor(255, 0, 0, fade), 6))

            r = (dist / self.max_range) * radius
            rad = math.radians(180 - ang)
            x = cx + r * math.cos(rad)
            y = cy - r * math.sin(rad)
            p.drawPoint(int(x), int(y))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Projet Suivi Cible - App")
        self.resize(1200, 740)

        # Serial
        self.ser = None
        self.parser = SerialParser()
        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history = deque(maxlen=5)

        # Radar->camera state machine
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

        # Camera
        self.cam = None

        # OpenCV detector (ton "folder OpenCV" = la détection ici)
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        self.detector_frame_count = 0
        self.target_start = None  # timer HOLD_SECONDS

        # UI
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        top = QHBoxLayout()
        self.port_combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)

        self.view_combo = QComboBox()
        self.view_combo.addItems(["Both", "Radar only", "Camera only"])
        self.view_combo.currentTextChanged.connect(self.apply_view_mode)

        self.auto_cam_check = QCheckBox("Auto camera from radar (2s)")
        self.auto_cam_check.setChecked(True)

        self.detect_check = QCheckBox("OpenCV detect")
        self.detect_check.setChecked(True)

        self.btn_camera = QPushButton("Start camera")
        self.btn_camera.clicked.connect(self.toggle_camera)

        top.addWidget(QLabel("Port:"))
        top.addWidget(self.port_combo)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_connect)
        top.addSpacing(20)
        top.addWidget(QLabel("View:"))
        top.addWidget(self.view_combo)
        top.addSpacing(20)
        top.addWidget(self.auto_cam_check)
        top.addWidget(self.detect_check)
        top.addWidget(self.btn_camera)
        top.addStretch()
        main_layout.addLayout(top)

        content = QHBoxLayout()

        self.radar_box = QGroupBox("Radar")
        radar_layout = QVBoxLayout(self.radar_box)
        self.radar_widget = RadarWidget()
        radar_layout.addWidget(self.radar_widget)

        self.debug_label = QLabel("")
        self.debug_label.setStyleSheet("color:#bbb; font-family:Consolas; font-size:12px;")
        self.debug_label.setVisible(SHOW_DEBUG)
        radar_layout.addWidget(self.debug_label)

        self.cam_box = QGroupBox("Camera")
        cam_layout = QVBoxLayout(self.cam_box)
        self.cam_label = QLabel("Camera stopped")
        self.cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cam_label.setStyleSheet("background:#111; color:white; padding:10px;")
        self.cam_label.setMinimumSize(520, 380)
        self.cam_label.setScaledContents(True)
        cam_layout.addWidget(self.cam_label)

        content.addWidget(self.radar_box, 1)
        content.addWidget(self.cam_box, 1)
        main_layout.addLayout(content)

        self.statusBar().showMessage("Ready")

        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.serial_tick)
        self.serial_timer.start(20)

        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.camera_tick)
        self.camera_timer.start(30)

        self.refresh_ports()
        self.apply_view_mode()

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_com_ports()
        if not ports:
            self.port_combo.addItem("No ports found", "")
            return
        for dev, desc in ports:
            self.port_combo.addItem(f"{dev} - {desc}", dev)

    def toggle_connect(self):
        if self.ser is None:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Port", "Aucun port COM sélectionné.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=SERIAL_TIMEOUT_SEC)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.parser = SerialParser()
            self.btn_connect.setText("Disconnect")
            self.statusBar().showMessage(f"Connected to {port} @ {BAUD}")
        except PermissionError:
            QMessageBox.critical(
                self, "Serial",
                f"Accès refusé à {port}.\n\nFerme Arduino Serial Monitor / VS Code Serial Monitor / autre script Python."
            )
        except Exception as e:
            QMessageBox.critical(self, "Serial", f"Impossible d'ouvrir {port}\n\n{e}")

    def disconnect_serial(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn_connect.setText("Connect")
        self.statusBar().showMessage("Disconnected")
        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history.clear()
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None
        self.debug_label.setText("")

    def apply_view_mode(self):
        mode = self.view_combo.currentText()
        self.radar_box.setVisible(mode in ["Both", "Radar only"])
        self.cam_box.setVisible(mode in ["Both", "Camera only"])

    # ---- Camera control ----
    def toggle_camera(self):
        if self.cam is None:
            self.open_camera()
        else:
            self.close_camera()

    def open_camera(self):
        if self.cam is not None:
            return
        cam = cv2.VideoCapture(CAM_INDEX)
        if not cam.isOpened():
            QMessageBox.critical(self, "Camera", "Caméra non accessible (index 0).")
            return
        self.cam = cam
        self.btn_camera.setText("Stop camera")
        self.cam_label.setText("Camera running")
        self.target_start = None

    def close_camera(self):
        try:
            if self.cam is not None:
                self.cam.release()
        except Exception:
            pass
        self.cam = None
        self.btn_camera.setText("Start camera")
        self.cam_label.setText("Camera stopped")
        self.target_start = None

    # ---- OpenCV processing INSIDE the app ----
    def camera_tick(self):
        if self.cam is None:
            return

        ret, frame = self.cam.read()
        if not ret:
            return

        now = time.time()

        detected_obj = False
        elapsed = 0.0

        if self.detect_check.isChecked() and not self.face_cascade.empty():
            self.detector_frame_count += 1

            if self.detector_frame_count % DETECT_EVERY_N_FRAMES == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4)

                detected_obj = (len(faces) > 0)

                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                if detected_obj:
                    if self.target_start is None:
                        self.target_start = now
                    elapsed = now - self.target_start
                else:
                    self.target_start = None
                    elapsed = 0.0

        # overlay texte (comme ton main.py)
        dist_text = f"Distance: {self.last_distance:.1f} cm" if self.last_distance is not None else "Distance: N/A"
        cv2.putText(frame, dist_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        if detected_obj and elapsed >= HOLD_SECONDS:
            cv2.putText(frame, "CIBLE DETECTEE", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        else:
            cv2.putText(frame, f"Hold: {elapsed:.1f}s / {HOLD_SECONDS:.1f}s", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        self.cam_label.setPixmap(frame_to_pixmap(frame))

    # ---- Serial / Radar ----
    def serial_tick(self):
        if self.ser is None:
            return

        try:
            if getattr(self.ser, "in_waiting", 0) <= 0:
                return
            raw = self.ser.readline()
            line = raw.decode(errors="replace").strip()
        except Exception:
            return

        a, d = self.parser.feed(line)

        if a is not None:
            self.last_angle_raw = a

        if d is not None:
            self.dist_history.append(d)
            valid = [x for x in self.dist_history if x is not None]
            smooth = statistics.median(valid) if valid else None
            if smooth is not None:
                self.last_distance = smooth

        self.radar_widget.update_data(self.last_angle_raw, self.last_distance)

        if SHOW_DEBUG:
            self.debug_label.setText(
                f"RAW: {line}\nparsed: angle={a}  dist={d}\nkept: angle_raw={self.last_angle_raw}  dist_smooth={self.last_distance}"
            )

        if not self.auto_cam_check.isChecked():
            return

        now = time.time()
        detected_now = (self.last_distance is not None) and (self.last_distance <= ON_THRESHOLD_CM)
        lost_now = (self.last_distance is None) or (self.last_distance >= OFF_THRESHOLD_CM)

        if self.state == "idle":
            if detected_now:
                if self.detect_start is None:
                    self.detect_start = now
                elif (now - self.detect_start) >= TRIGGER_CONFIRM_SEC:
                    self.state = "open"
                    self.detect_start = None
                    self.loss_start = None
                    self.statusBar().showMessage("Radar confirmed -> opening camera")
                    self.open_camera()
            else:
                self.detect_start = None

        elif self.state == "open":
            if lost_now:
                if self.loss_start is None:
                    self.loss_start = now
                elif (now - self.loss_start) >= CAM_CLOSE_AFTER_RADAR_LOSS:
                    self.statusBar().showMessage("Radar lost -> closing camera")
                    self.close_camera()
                    self.state = "idle"
                    self.detect_start = None
                    self.loss_start = None
            else:
                self.loss_start = None

    def closeEvent(self, event):
        self.close_camera()
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()