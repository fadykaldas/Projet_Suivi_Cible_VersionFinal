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
RADAR_MAX_RANGE_CM = 200.0      # échelle du radar (ajuste selon ton capteur)
POINT_TTL_SEC = 2.0             # durée de vie des points rouges
SWEEP_TRAIL = 18                # traînée du balayage

# ---- Calibration angle (IMPORTANT) ----
# Mets ici la VRAIE plage de ton servo si ce n’est pas 0..180
SERVO_MIN_ANGLE = 0.0           # ex: 20.0
SERVO_MAX_ANGLE = 180.0         # ex: 160.0

RADAR_FLIP = True               # True si gauche/droite est inversé, sinon False
RADAR_OFFSET_DEG = 0.0          # ex: +10 ou -10 si ton centre est décalé
# --------------------------------------
# ------------------------


def list_com_ports():
    ports = list(serial.tools.list_ports.comports())
    return [(p.device, p.description) for p in ports]


def parse_angle_distance(line: str):
    """
    Supporte:
    - "53.2"
    - "Distance: 53.2 cm"
    - "Angle: 90 Distance: 53.2"
    - "90,53.2"
    Retour:
    - si 2+ nombres -> angle = 1er, distance = dernier
    - si 1 nombre -> distance seulement (angle None)
    """
    if not line:
        return None, None

    nums = re.findall(r"\d+(?:\.\d+)?", line)
    if not nums:
        return None, None

    if len(nums) >= 2:
        return float(nums[0]), float(nums[-1])
    return None, float(nums[0])


def frame_to_pixmap(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    bytes_per_line = ch * w
    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
    return QPixmap.fromImage(qimg)


class RadarWidget(QWidget):
    """
    Radar style:
    - demi-cercle
    - grille verte
    - balayage vert (avec traînée)
    - points rouges (objets)
    """
    def __init__(self):
        super().__init__()
        self.setMinimumSize(520, 380)

        self.max_range = float(RADAR_MAX_RANGE_CM)
        self.last_angle = 90.0
        self.last_distance = None

        self.sweep_angles = deque(maxlen=SWEEP_TRAIL)          # angles récents (trail)
        self.points = deque(maxlen=400)                        # (angle, dist, timestamp)

    def set_max_range(self, cm: float):
        self.max_range = max(10.0, float(cm))
        self.update()

    def map_angle(self, angle_deg: float) -> float:
        """
        Convertit l’angle servo -> angle radar 0..180
        - clamp sur SERVO_MIN..SERVO_MAX
        - normalize en 0..180 si servo ne fait pas tout le range
        - offset
        - flip optionnel
        """
        a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle_deg)))

        if SERVO_MAX_ANGLE != SERVO_MIN_ANGLE:
            a = (a - SERVO_MIN_ANGLE) * 180.0 / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)

        a = a + RADAR_OFFSET_DEG
        a = max(0.0, min(180.0, a))

        if RADAR_FLIP:
            a = 180.0 - a

        return a

    def update_data(self, angle, dist):
        now = time.time()

        # angle
        if angle is not None:
            angle = self.map_angle(angle)
            self.last_angle = angle
            self.sweep_angles.appendleft(angle)
        else:
            # si pas d’angle, on garde le dernier angle
            self.sweep_angles.appendleft(self.last_angle)

        # distance
        if dist is not None:
            self.last_distance = float(dist)
            if 0.0 < self.last_distance <= self.max_range:
                self.points.append((self.last_angle, self.last_distance, now))

        self.update()

    def paintEvent(self, event):
        # cleanup points expirés
        now = time.time()
        while self.points and (now - self.points[0][2]) > POINT_TTL_SEC:
            self.points.popleft()

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # fond
        p.fillRect(self.rect(), QColor(0, 0, 0))

        w = self.width()
        h = self.height()

        # centre en bas
        cx = w * 0.5
        cy = h * 0.95
        radius = min(w * 0.48, h * 0.9)

        green = QColor(0, 255, 80)
        dark_green = QColor(0, 110, 40)

        # grille: arcs
        pen_grid = QPen(dark_green)
        pen_grid.setWidth(2)
        p.setPen(pen_grid)

        for frac in [0.25, 0.5, 0.75, 1.0]:
            r = radius * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0 * 16, 180 * 16)

        # grille: lignes d’angles
        for ang in [0, 30, 60, 90, 120, 150, 180]:
            rad = math.radians(180 - ang)  # 0=left, 180=right
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(int(cx), int(cy), int(x), int(y))

        # textes
        p.setPen(QPen(green))
        p.setFont(QFont("Consolas", 10))
        p.drawText(10, 20, "Radar Display")

        # labels distance
        for frac in [0.5, 1.0]:
            r = radius * frac
            cm = self.max_range * frac
            p.drawText(int(cx + 10), int(cy - r - 5), f"{cm:.0f} cm")

        angle_txt = f"Angle: {self.last_angle:.0f}°"
        dist_txt = f"Distance: {self.last_distance:.0f} cm" if self.last_distance is not None else "Distance: N/A"
        p.drawText(10, 40, angle_txt)
        p.drawText(10, 60, dist_txt)

        # balayage (trail)
        for i, ang in enumerate(self.sweep_angles):
            alpha = max(20, 255 - i * 12)
            pen_sweep = QPen(QColor(0, 255, 80, alpha))
            pen_sweep.setWidth(2 if i > 0 else 4)
            p.setPen(pen_sweep)

            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        # points rouges (objets)
        for (ang, dist, ts) in self.points:
            age = now - ts
            fade = max(40, int(255 * (1.0 - age / POINT_TTL_SEC)))
            pen_pt = QPen(QColor(255, 0, 0, fade))
            pen_pt.setWidth(6)
            p.setPen(pen_pt)

            r = (dist / self.max_range) * radius
            rad = math.radians(180 - ang)
            x = cx + r * math.cos(rad)
            y = cy - r * math.sin(rad)
            p.drawPoint(int(x), int(y))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Projet Suivi Cible - App")
        self.resize(1200, 700)

        # Serial + data
        self.ser = None
        self.last_angle_raw = None      # angle brut Arduino
        self.last_distance = None
        self.dist_history = deque(maxlen=5)

        # Radar->camera state machine
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

        # Camera
        self.cam = None

        # --- UI ---
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

        angle, dist = parse_angle_distance(line)

        if angle is not None:
            self.last_angle_raw = angle

        self.dist_history.append(dist)
        valid = [d for d in self.dist_history if d is not None]
        smooth = statistics.median(valid) if valid else None
        if smooth is not None:
            self.last_distance = smooth

        # update radar (RadarWidget mappe l’angle lui-même)
        self.radar_widget.update_data(self.last_angle_raw, self.last_distance)

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