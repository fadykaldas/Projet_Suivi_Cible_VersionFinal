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

# Déclenchement caméra
ON_THRESHOLD_CM = 60.0
OFF_THRESHOLD_CM = 70.0
TRIGGER_CONFIRM_SEC = 2.0
CAM_CLOSE_AFTER_RADAR_LOSS = 10.0

CAM_INDEX = 0

# Radar rendu
RADAR_MAX_RANGE_CM = 200.0      # échelle du radar
POINT_TTL_SEC = 2.0             # durée de vie des points rouges
SWEEP_TRAIL = 18                # traînée du balayage

# ---- Calibration angle ----
# Mets ici la VRAIE plage de ton servo si ce n’est pas 0..180
SERVO_MIN_ANGLE = 0.0           # ex: 20.0
SERVO_MAX_ANGLE = 180.0         # ex: 160.0
RADAR_FLIP = True               # inverse gauche/droite si besoin
RADAR_OFFSET_DEG = 0.0          # décale le centre si besoin (+10 / -10)
# --------------------------

# Debug (affiche raw + angle/distance en bas)
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
    Rend l’app robuste aux formats Arduino:
    1) "angle,distance"
    2) "Angle: 161 Distance: 8"
    3) 2 lignes: "161" puis "8"
    4) distance seule (angle restera None)
    """
    def __init__(self):
        self.pending_angle = None  # pour le cas angle puis distance sur 2 lignes

    def feed(self, line: str):
        if not line:
            return None, None

        s = line.strip()
        low = s.lower()

        nums = re.findall(r"\d+(?:\.\d+)?", s)
        if not nums:
            return None, None

        # Cas "angle,distance" ou plusieurs nombres dans la même ligne
        if len(nums) >= 2:
            a = float(nums[0])
            d = float(nums[-1])
            self.pending_angle = None
            return a, d

        # Cas 1 seul nombre
        n = float(nums[0])

        # Si la ligne mentionne angle/deg -> angle
        if ("ang" in low) or ("deg" in low):
            self.pending_angle = n
            return n, None

        # Si la ligne mentionne distance/cm -> distance
        if ("dist" in low) or ("cm" in low):
            if self.pending_angle is not None:
                a = self.pending_angle
                self.pending_angle = None
                return a, n
            return None, n

        # Cas lignes numériques pures:
        # on assume "angle puis distance" en alternance.
        if self.pending_angle is None:
            self.pending_angle = n
            return None, None
        else:
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

        self.sweep_angles = deque(maxlen=SWEEP_TRAIL)          # angles récents (trail)
        self.points = deque(maxlen=400)                        # (angle, dist, timestamp)

    def map_angle(self, angle_deg: float) -> float:
        a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle_deg)))

        # normalize en 0..180 si servo ne fait pas tout le range
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
            # si pas d’angle, on garde le dernier angle (mais ça veut dire Arduino n’envoie pas d’angle)
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

        w = self.width()
        h = self.height()

        cx = w * 0.5
        cy = h * 0.95
        radius = min(w * 0.48, h * 0.9)

        green = QColor(0, 255, 80)
        dark_green = QColor(0, 110, 40)

        # grille arcs
        p.setPen(QPen(dark_green, 2))
        for frac in [0.25, 0.5, 0.75, 1.0]:
            r = radius * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0 * 16, 180 * 16)

        # lignes angle
        for ang in [0, 30, 60, 90, 120, 150, 180]:
            rad = math.radians(180 - ang)  # 0=left 180=right
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(int(cx), int(cy), int(x), int(y))

        # textes
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

        # sweep trail
        for i, ang in enumerate(self.sweep_angles):
            alpha = max(20, 255 - i * 12)
            pen = QPen(QColor(0, 255, 80, alpha), 4 if i == 0 else 2)
            p.setPen(pen)

            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        # points rouges
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
        self.resize(1200, 720)

        self.ser = None
        self.parser = SerialParser()

        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history = deque(maxlen=5)

        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

        self.cam = None

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Top bar
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
        top.addWidget(self.btn_camera)
        top.addStretch()
        main_layout.addLayout(top)

        # Content
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

        # Timers
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

    # Camera
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

    def close_camera(self):
        try:
            if self.cam is not None:
                self.cam.release()
        except Exception:
            pass
        self.cam = None
        self.btn_camera.setText("Start camera")
        self.cam_label.setText("Camera stopped")

    def camera_tick(self):
        if self.cam is None:
            return
        ret, frame = self.cam.read()
        if not ret:
            return
        self.cam_label.setPixmap(frame_to_pixmap(frame))

    # Serial / Radar
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

        # si on reçoit angle seul -> on garde
        if a is not None:
            self.last_angle_raw = a

        # smoothing distance
        if d is not None:
            self.dist_history.append(d)
            valid = [x for x in self.dist_history if x is not None]
            smooth = statistics.median(valid) if valid else None
            if smooth is not None:
                self.last_distance = smooth

        # update radar (même si angle ou distance arrive séparément)
        self.radar_widget.update_data(self.last_angle_raw, self.last_distance)

        if SHOW_DEBUG:
            self.debug_label.setText(
                f"RAW: {line}\nparsed: angle={a}  dist={d}\nkept: angle_raw={self.last_angle_raw}  dist_smooth={self.last_distance}"
            )

        # auto camera
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