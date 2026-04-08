import sys
import time
import re
import math
import json
import statistics
from dataclasses import dataclass, asdict
from pathlib import Path
from datetime import datetime
from collections import deque

import cv2
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QTimer, QPointF, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont, QAction
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QFrame, QMessageBox, QStackedWidget,
    QCheckBox, QSlider, QSpinBox, QDoubleSpinBox, QFileDialog,
    QLineEdit, QCompleter,
    QGridLayout
)

from image_3d import Image3DConfig, SingleCameraChessboardMeasurer

# -------------------- CONFIG / PERSISTENCE --------------------
APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "app_config.json"
SCREENSHOT_DIR = APP_DIR / "screenshots"


@dataclass
class AppConfig:
    baud: int = 115200
    serial_timeout_sec: float = 0.05

    on_threshold_cm: float = 60.0
    off_threshold_cm: float = 70.0
    trigger_confirm_sec: float = 2.0
    cam_close_after_radar_loss: float = 10.0

    cam_index: int = 0
    hold_seconds: float = 2.0
    detect_every_n_frames: int = 2

    radar_max_range_cm: float = 40.0
    point_ttl_sec: float = 2.0
    sweep_trail: int = 18

    servo_min_angle: float = 0.0
    servo_max_angle: float = 180.0
    radar_flip: bool = True
    radar_offset_deg: float = 0.0

    auto_camera_from_radar: bool = True
    opencv_detect: bool = True

    target_object: str = "cell phone"
    object_confidence: float = 0.25

    trajectory_enabled: bool = True
    trajectory_history_len: int = 12
    trajectory_prediction_steps: int = 6
    trajectory_step_sec: float = 0.20
    trajectory_min_dt: float = 0.001

    image3d_chessboard_cols: int = 9
    image3d_chessboard_rows: int = 6
    image3d_square_size_mm: float = 25.0
    image3d_min_calibration_frames: int = 12
    image3d_blur_kernel_size: int = 5
    image3d_min_contour_area: int = 2000
    image3d_debug: bool = False

    def save(self, path: Path = CONFIG_PATH):
        path.write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")

    @staticmethod
    def load(path: Path = CONFIG_PATH):
        if not path.exists():
            return AppConfig()
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            cfg = AppConfig()
            for k, v in data.items():
                if hasattr(cfg, k):
                    setattr(cfg, k, v)
            return cfg
        except Exception:
            return AppConfig()


# -------------------- THEME (PRO LOOK) --------------------
def apply_pro_theme(app: QApplication):
    app.setStyleSheet("""
    * { font-family: Segoe UI; font-size: 13px; }
    QMainWindow { background: #0b0f17; }

    QFrame#TopBar {
        background: #0b1220;
        border-bottom: 1px solid #1f2a44;
    }
    QLabel#AppTitle {
        color: #e5e7eb;
        font-size: 16px;
        font-weight: 800;
    }
    QLabel#AppSubTitle { color: #94a3b8; }
    QLabel#Pill {
        padding: 4px 10px;
        border-radius: 999px;
        background: #0f172a;
        border: 1px solid #1f2a44;
        color: #cbd5e1;
        font-weight: 600;
    }
    QLabel#PillGreen { background: #052e1a; border: 1px solid #14532d; color: #86efac; }
    QLabel#PillRed { background: #2a0f12; border: 1px solid #7f1d1d; color: #fecaca; }

    QFrame#Sidebar {
        background: #0f172a;
        border-right: 1px solid #1f2a44;
    }
    QPushButton#NavBtn {
        text-align: left;
        padding: 10px 12px;
        border: 0px;
        border-radius: 12px;
        color: #dbeafe;
        background: transparent;
        font-weight: 650;
    }
    QPushButton#NavBtn:hover { background: #172554; }
    QPushButton#NavBtn:checked { background: #1d4ed8; }
    QLabel#SidebarSmall { color: #94a3b8; font-size: 12px; }

    QFrame#Card {
        background: #0f172a;
        border: 1px solid #1f2a44;
        border-radius: 18px;
    }
    QLabel#CardTitle {
        color: #e5e7eb;
        font-size: 14px;
        font-weight: 800;
    }
    QLabel { color: #cbd5e1; }

    QComboBox, QSpinBox, QDoubleSpinBox {
        background: #0b1220;
        border: 1px solid #24324f;
        padding: 6px 10px;
        border-radius: 12px;
        color: #e5e7eb;
    }
    QComboBox::drop-down { border: 0; }
    QComboBox QAbstractItemView {
        background: #0b1220;
        border: 1px solid #24324f;
        selection-background-color: #1d4ed8;
        color: #e5e7eb;
        padding: 6px;
    }

    QPushButton {
        background: #1d4ed8;
        color: white;
        border: 0px;
        padding: 8px 12px;
        border-radius: 12px;
        font-weight: 700;
    }
    QPushButton:hover { background: #2563eb; }
    QPushButton:pressed { background: #1e40af; }
    QPushButton:disabled { background: #334155; color: #94a3b8; }

    QPushButton#SecondaryBtn {
        background: #0b1220;
        color: #e5e7eb;
        border: 1px solid #24324f;
    }
    QPushButton#SecondaryBtn:hover { background: #111a2e; }

    QCheckBox { color: #e5e7eb; font-weight: 600; }
    QCheckBox::indicator { width: 18px; height: 18px; }
    QCheckBox::indicator:unchecked {
        border: 1px solid #334155; border-radius: 6px; background: #0b1220;
    }
    QCheckBox::indicator:checked {
        border: 1px solid #1d4ed8; border-radius: 6px; background: #1d4ed8;
    }

    QSlider::groove:horizontal {
        height: 8px; border-radius: 999px; background: #1f2a44;
    }
    QSlider::handle:horizontal {
        width: 18px; margin: -6px 0; border-radius: 9px; background: #1d4ed8;
    }

    QStatusBar { background: #0b1220; color: #94a3b8; border-top: 1px solid #1f2a44; }
    """)


# -------------------- UTILS --------------------
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
            self.pending_angle = None
            return float(nums[0]), float(nums[-1])

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

        if self.pending_angle is None:
            self.pending_angle = n
            return None, None
        a = self.pending_angle
        self.pending_angle = None
        return a, n


# -------------------- RADAR WIDGET --------------------
class RadarWidget(QWidget):
    def __init__(self, cfg: AppConfig):
        super().__init__()
        self.cfg = cfg
        self.setMinimumSize(520, 380)

        self.last_angle = 90.0
        self.last_distance = None

        self.sweep_angles = deque(maxlen=self.cfg.sweep_trail)
        self.points = deque(maxlen=600)  # (angle, dist, timestamp)

    def update_config(self, cfg: AppConfig):
        self.cfg = cfg
        self.sweep_angles = deque(self.sweep_angles, maxlen=self.cfg.sweep_trail)
        self.update()

    def map_angle(self, angle_deg: float) -> float:
        a = max(self.cfg.servo_min_angle, min(self.cfg.servo_max_angle, float(angle_deg)))
        if self.cfg.servo_max_angle != self.cfg.servo_min_angle:
            a = (a - self.cfg.servo_min_angle) * 180.0 / (self.cfg.servo_max_angle - self.cfg.servo_min_angle)
        a = a + self.cfg.radar_offset_deg
        a = max(0.0, min(180.0, a))
        if self.cfg.radar_flip:
            a = 180.0 - a
        return a

    def update_data(self, angle_raw, dist_cm):
        now = time.time()

        if angle_raw is not None:
            self.last_angle = self.map_angle(angle_raw)
            self.sweep_angles.appendleft(self.last_angle)
        else:
            self.sweep_angles.appendleft(self.last_angle)

        if dist_cm is not None:
            self.last_distance = float(dist_cm)
            if 0.0 < self.last_distance <= self.cfg.radar_max_range_cm:
                self.points.append((self.last_angle, self.last_distance, now))

        self.update()

    def paintEvent(self, event):
        now = time.time()
        while self.points and (now - self.points[0][2]) > self.cfg.point_ttl_sec:
            self.points.popleft()

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(18, 20, 25))

        w, h = self.width(), self.height()
        margin = 14
        baseline_y = h - 28
        cx, cy = w * 0.5, baseline_y
        radius = min((w * 0.5) - margin, (h * 0.86))

        grid_neon = QColor(71, 255, 63, 210)
        grid_soft = QColor(71, 255, 63, 110)

        # Bottom baseline like the Processing radar UI.
        p.setPen(QPen(grid_neon, 2))
        p.drawLine(margin, int(cy), w - margin, int(cy))

        # Range arcs.
        p.setPen(QPen(grid_neon, 2))
        ring_fracs = [0.25, 0.50, 0.75, 1.0]
        for frac in ring_fracs:
            r = radius * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0, 180 * 16)

        # Radial grid at 30-degree steps, labels excluding 0 and 180 for a cleaner look.
        p.setPen(QPen(grid_soft, 2))
        p.setFont(QFont("Consolas", 11, QFont.Weight.Bold))
        for ang in [0, 30, 60, 90, 120, 150, 180]:
            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(int(cx), int(cy), int(x), int(y))

            if ang in (30, 60, 90, 120, 150):
                lx = cx + (radius + 18) * math.cos(rad)
                ly = cy - (radius + 18) * math.sin(rad)
                p.setPen(QPen(grid_neon, 2))
                p.drawText(int(lx - 18), int(ly + 6), f"{ang}°")
                p.setPen(QPen(grid_soft, 2))

        # Green sweep fan trail (newest first).
        for i, ang in enumerate(self.sweep_angles):
            alpha = max(8, 220 - i * 14)
            width = 8 if i == 0 else 6
            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.setPen(QPen(QColor(38, 255, 72, alpha), width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        # Echoes as red radial hits (like Processing red spikes).
        for (ang, dist, ts) in self.points:
            age = now - ts
            ttl = max(0.001, self.cfg.point_ttl_sec)
            life = max(0.0, 1.0 - (age / ttl))
            alpha = int(40 + 180 * life)
            thickness = 3.0 + 3.0 * life

            r = (dist / self.cfg.radar_max_range_cm) * radius
            r = max(0.0, min(radius, r))
            rad = math.radians(180 - ang)
            x = cx + r * math.cos(rad)
            y = cy - r * math.sin(rad)

            p.setPen(QPen(QColor(170, 0, 0, alpha), thickness, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        # Text labels at the bottom, matching the screenshot style.
        p.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        p.setPen(QPen(grid_neon, 1))

        p.drawText(margin + 6, h - 6, "Projet Suivi Cibles")
        p.drawText(int(cx + 16), h - 6, f"Angle: {self.last_angle:.0f}°")

        if self.last_distance is None:
            dist_txt = "Distance: N/A"
        else:
            dist_txt = f"Distance: {self.last_distance:.0f}cm"
        p.drawText(int(w * 0.73), h - 6, dist_txt)

        # Bottom range markers (10cm, 20cm, ... scaled to configured max range).
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        max_range = max(1.0, float(self.cfg.radar_max_range_cm))
        for frac in ring_fracs:
            r = radius * frac
            value = int(round(max_range * frac))
            label = f"{value}cm"
            p.drawText(int(cx + r - 18), int(cy - 6), label)


class CameraCaptureThread(QThread):
    frame_ready = pyqtSignal(object)
    camera_error = pyqtSignal(str)

    def __init__(self, cam_index: int):
        super().__init__()
        self.cam_index = int(cam_index)
        self._running = True

    def stop(self):
        self._running = False

    def _open_camera(self):
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, None):
            try:
                if backend is None:
                    cam = cv2.VideoCapture(self.cam_index)
                else:
                    cam = cv2.VideoCapture(self.cam_index, backend)
                if cam is not None and cam.isOpened():
                    try:
                        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                    return cam
                if cam is not None:
                    cam.release()
            except Exception:
                try:
                    if cam is not None:
                        cam.release()
                except Exception:
                    pass
        return None

    def run(self):
        cam = self._open_camera()
        if cam is None or not cam.isOpened():
            self.camera_error.emit(f"Camera not accessible (index {self.cam_index}).")
            return

        try:
            while self._running:
                ret, frame = cam.read()
                if ret and frame is not None:
                    self.frame_ready.emit(frame)
                self.msleep(12)
        except Exception as e:
            self.camera_error.emit(f"Camera capture error: {e}")
        finally:
            try:
                cam.release()
            except Exception:
                pass


# -------------------- UI BLOCKS --------------------
def card(title: str, body: QWidget) -> QFrame:
    c = QFrame()
    c.setObjectName("Card")
    lay = QVBoxLayout(c)
    lay.setContentsMargins(16, 16, 16, 16)
    lay.setSpacing(10)
    t = QLabel(title)
    t.setObjectName("CardTitle")
    lay.addWidget(t)
    lay.addWidget(body, 1)
    return c


# -------------------- MAIN WINDOW --------------------
class MainWindow(QMainWindow):
    def goto_measure(self):
        self.pages.setCurrentIndex(3)
        self.lbl_title.setText("Mesure d'objet")
        self.lbl_sub.setText("Mesure avancée : calibration, contours, surface")
        self._set_nav_checked("btn_measure")
    def __init__(self):
        super().__init__()
        self.cfg = AppConfig.load()

        self.setWindowTitle("Projet Suivi Cible")
        self.resize(1400, 900)

        # runtime state
        self.ser = None
        self.parser = SerialParser()
        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history = deque(maxlen=5)
        self.serial_buffer = ""
        self.last_real_angle_ts = 0.0
        self.synthetic_angle_raw = 90.0
        self.synthetic_dir = 1.0
        self.last_synth_update_ts = time.time()

        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

        self.cam = None
        self.camera_thread = None
        self.latest_frame = None
        self.last_frame_bgr = None  # <- for saving camera snapshot
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

        # YOLOv8 object detection (used for OpenCV tab)
        self.yolo = None
        if YOLO is not None:
            try:
                model_path = Path(__file__).resolve().parents[2] / "yolov8n.pt"
                if model_path.exists():
                    self.yolo = YOLO(str(model_path))
                else:
                    self.yolo = YOLO("yolov8n.pt")
            except Exception as e:
                self.statusBar().showMessage(f"YOLO model load failed: {e}")
        else:
            self.statusBar().showMessage("ultralytics (YOLO) not installed; using Haar cascade fallback")

        self.target_object = self.cfg.target_object

        # Buzzer (Arduino) cooldown
        self.last_buzz_time = 0.0
        self.buzzer_cooldown = 2.0

        self.detector_frame_count = 0
        self.target_start = None
        self.object_detection_running = False
        self.object_detection_cap = None
        self.object_detection_timer = None

        self.image3d_running = False
        self.image3d_cap = None
        self.image3d_timer = None
        self.image3d_debug_images = {}
        self.image3d_backend = SingleCameraChessboardMeasurer(
            self._build_image3d_config(),
            APP_DIR / "calibration_data.npz",
        )

        # Keep latest inference boxes for a short time to avoid flicker when running detection every N frames
        self.last_det_boxes = []  # list of (x1,y1,x2,y2,label,is_target)
        self.last_det_ts = 0.0
        self.det_box_ttl = 0.6  # seconds

        # Predictive trajectory state
        self.target_track = deque(maxlen=self.cfg.trajectory_history_len)  # (cx, cy, t)
        self.predicted_points = []
        self.last_target_center = None

        self._build_menubar()

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        sidebar = self._build_sidebar()
        root_layout.addWidget(sidebar, 0)

        main = QWidget()
        main_layout = QVBoxLayout(main)
        main_layout.setContentsMargins(18, 14, 18, 14)
        main_layout.setSpacing(12)

        self.topbar = self._build_topbar()
        main_layout.addWidget(self.topbar)

        self.pages = QStackedWidget()

        self.page_home = self._build_home_page()
        self.page_live = self._build_live_page()
        self.page_face_tracking = self._build_face_tracking_page()
        self.page_measure = self._build_measure_page()
        self.page_image_3d = self._build_image_3d_page()
        self.page_object_detection = self._build_object_detection_page()
        self.page_settings = self._build_settings_page()
        self.page_about = self._build_about_page()

        self.pages.addWidget(self.page_home)           # 0
        self.pages.addWidget(self.page_live)           # 1
        self.pages.addWidget(self.page_face_tracking)  # 2
        self.pages.addWidget(self.page_measure)        # 3
        self.pages.addWidget(self.page_image_3d)       # 4
        self.pages.addWidget(self.page_object_detection)  # 5
        self.pages.addWidget(self.page_settings)       # 6
        self.pages.addWidget(self.page_about)          # 7

        main_layout.addWidget(self.pages, 1)
        root_layout.addWidget(main, 1)


        self.statusBar().showMessage("Ready")

        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.serial_tick)
        self.serial_timer.start(20)

        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.camera_tick)
        self.camera_timer.start(30)

        self.refresh_ports()
        self._apply_cfg_to_ui()
        self.home_timer = QTimer()
        self.home_timer.timeout.connect(self._refresh_home_dashboard)
        self.home_timer.start(500)
        self._refresh_home_dashboard()
        self.goto_home()

    def _get_object_list(self):
        """Return a list of object names to choose from for target detection."""
        if self.yolo is not None:
            try:
                return list(self.yolo.names.values())
            except Exception:
                pass
        # Fallback list (COCO-like keywords)
        return [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
            "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
            "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
            "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
            "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
            "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
            "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush"
        ]

    # ---------- MENUBAR ----------
    def _build_menubar(self):
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        view_menu = menubar.addMenu("View")
        help_menu = menubar.addMenu("Help")

        self.act_screenshot = QAction("Screenshot (auto)", self)
        self.act_screenshot.setShortcut("Ctrl+Shift+S")
        self.act_screenshot.triggered.connect(self.take_screenshot_auto)
        file_menu.addAction(self.act_screenshot)

        self.act_screenshot_as = QAction("Screenshot As...", self)
        self.act_screenshot_as.triggered.connect(self.take_screenshot_as)
        file_menu.addAction(self.act_screenshot_as)

        file_menu.addSeparator()

        act_open_cfg = QAction("Open config...", self)
        act_open_cfg.triggered.connect(self.open_config_dialog)
        file_menu.addAction(act_open_cfg)

        act_save_cfg = QAction("Save config", self)
        act_save_cfg.triggered.connect(self.save_config)
        file_menu.addAction(act_save_cfg)

        file_menu.addSeparator()

        act_exit = QAction("Exit", self)
        act_exit.triggered.connect(self.close)
        file_menu.addAction(act_exit)

        view_menu.addAction(QAction("Home", self, triggered=self.goto_home))
        view_menu.addAction(QAction("Live", self, triggered=self.goto_live))
        view_menu.addAction(QAction("Image 3D", self, triggered=self.goto_image_3d))
        view_menu.addAction(QAction("Settings", self, triggered=self.goto_settings))
        help_menu.addAction(QAction("About", self, triggered=self.goto_about))

    # ---------- SIDEBAR ----------
    def _nav_button(self, text: str, on_click):
        b = QPushButton(text)
        b.setObjectName("NavBtn")
        b.setCheckable(True)
        b.clicked.connect(on_click)
        return b

    def _build_sidebar(self) -> QFrame:
        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        lay = QVBoxLayout(sidebar)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)

        title = QLabel("Suivi Cible")
        title.setObjectName("AppTitle")
        lay.addWidget(title)

        sub = QLabel("Arduino Radar + OpenCV")
        sub.setObjectName("SidebarSmall")
        lay.addWidget(sub)

        lay.addSpacing(8)

        self.btn_home = self._nav_button("  🏠  Home", self.goto_home)
        self.btn_live = self._nav_button("  📡  Live", self.goto_live)
        self.btn_face_tracking = self._nav_button("  🎯  Face Tracking", self.goto_face_tracking)
        self.btn_measure = self._nav_button("  📏  Mesure d'objet", self.goto_measure)
        self.btn_image_3d = self._nav_button("  🧊  Image 3D", self.goto_image_3d)
        self.btn_object_detection = self._nav_button("  🔎  Object Detection", self.goto_object_detection)
        self.btn_settings = self._nav_button("  ⚙️  Settings", self.goto_settings)
        self.btn_about = self._nav_button("  ℹ️  About", self.goto_about)

        lay.addWidget(self.btn_home)
        lay.addWidget(self.btn_live)
        lay.addWidget(self.btn_face_tracking)
        lay.addWidget(self.btn_measure)
        lay.addWidget(self.btn_image_3d)
        lay.addWidget(self.btn_object_detection)
        lay.addWidget(self.btn_settings)
        lay.addWidget(self.btn_about)
        lay.addStretch()

        foot = QLabel("v1.1 • ProjetProg")
        foot.setObjectName("SidebarSmall")

        lay.addWidget(foot)
        return sidebar

    def _set_nav_checked(self, which: str):
        for b in [self.btn_home, self.btn_live, self.btn_face_tracking, self.btn_measure, self.btn_image_3d, self.btn_object_detection, self.btn_settings, self.btn_about]:
            b.setChecked(False)
        getattr(self, which).setChecked(True)

    def _build_image3d_config(self) -> Image3DConfig:
        """Create the backend config from the persisted application config."""
        return Image3DConfig(
            chessboard_cols=int(self.cfg.image3d_chessboard_cols),
            chessboard_rows=int(self.cfg.image3d_chessboard_rows),
            square_size_mm=float(self.cfg.image3d_square_size_mm),
            min_calibration_frames=int(self.cfg.image3d_min_calibration_frames),
            blur_kernel_size=int(self.cfg.image3d_blur_kernel_size),
            min_contour_area=int(self.cfg.image3d_min_contour_area),
        )

    def _refresh_image3d_backend(self, reload_saved: bool = True):
        """Rebuild the Image 3D backend after settings changes."""
        self.image3d_backend = SingleCameraChessboardMeasurer(
            self._build_image3d_config(),
            APP_DIR / "calibration_data.npz",
        )
        if not reload_saved:
            self.image3d_backend.reset_calibration_samples()

    def _build_measure_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.measure_label = QLabel("Camera stopped")
        self.measure_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.measure_label.setStyleSheet("background:#0b1220; border-radius:14px; color:#e5e7eb;")
        self.measure_label.setMinimumSize(520, 380)
        self.measure_label.setScaledContents(True)

        self.btn_measure_start = QPushButton("Start Mesure d'objet")
        self.btn_measure_start.setObjectName("SecondaryBtn")
        self.btn_measure_start.clicked.connect(self.toggle_measure_camera)

        lay.addWidget(card("Mesure d'objet (CamRuler)", self.measure_label), 1)
        lay.addWidget(self.btn_measure_start)
        return page

    def _build_image_3d_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        controls = QFrame()
        controls.setObjectName("Card")
        controls_lay = QGridLayout(controls)
        controls_lay.setContentsMargins(16, 16, 16, 16)
        controls_lay.setHorizontalSpacing(12)
        controls_lay.setVerticalSpacing(10)

        self.image3d_btn_start = QPushButton("Start Image 3D")
        self.image3d_btn_start.clicked.connect(self.toggle_image3d_camera)

        self.image3d_btn_capture = QPushButton("Capture Chessboard")
        self.image3d_btn_capture.setObjectName("SecondaryBtn")
        self.image3d_btn_capture.clicked.connect(self.capture_image3d_sample)

        self.image3d_btn_calibrate = QPushButton("Calibrate Camera")
        self.image3d_btn_calibrate.setObjectName("SecondaryBtn")
        self.image3d_btn_calibrate.clicked.connect(self.run_image3d_calibration)

        self.image3d_btn_reset = QPushButton("Reset Samples")
        self.image3d_btn_reset.setObjectName("SecondaryBtn")
        self.image3d_btn_reset.clicked.connect(self.reset_image3d_samples)

        self.image3d_btn_delete = QPushButton("Delete Saved Calibration")
        self.image3d_btn_delete.setObjectName("SecondaryBtn")
        self.image3d_btn_delete.clicked.connect(self.delete_image3d_calibration)

        self.image3d_chk_debug = QCheckBox("Debug processing")
        self.image3d_chk_debug.setChecked(bool(self.cfg.image3d_debug))

        self.image3d_status = QLabel("Calibration file: waiting")
        self.image3d_status.setStyleSheet("color:#94a3b8;")

        self.image3d_results = QLabel(
            "Width: N/A\nLength: N/A\nHeight: N/A\nRMS: N/A\nSamples: 0"
        )
        self.image3d_results.setStyleSheet("color:#cbd5e1;")

        controls_lay.addWidget(self.image3d_btn_start, 0, 0)
        controls_lay.addWidget(self.image3d_btn_capture, 0, 1)
        controls_lay.addWidget(self.image3d_btn_calibrate, 0, 2)
        controls_lay.addWidget(self.image3d_btn_reset, 0, 3)
        controls_lay.addWidget(self.image3d_btn_delete, 0, 4)
        controls_lay.addWidget(self.image3d_chk_debug, 1, 0, 1, 2)
        controls_lay.addWidget(self.image3d_status, 1, 2, 1, 3)
        controls_lay.addWidget(self.image3d_results, 2, 0, 1, 5)

        views = QHBoxLayout()
        views.setSpacing(12)

        self.image3d_label = QLabel("Image 3D stopped")
        self.image3d_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image3d_label.setStyleSheet("background:#0b1220; border-radius:14px; color:#e5e7eb;")
        self.image3d_label.setMinimumSize(640, 420)
        self.image3d_label.setScaledContents(True)

        debug_panel = QWidget()
        debug_lay = QVBoxLayout(debug_panel)
        debug_lay.setContentsMargins(0, 0, 0, 0)
        debug_lay.setSpacing(12)

        self.image3d_debug_threshold = QLabel("Debug threshold")
        self.image3d_debug_threshold.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image3d_debug_threshold.setStyleSheet("background:#0b1220; border-radius:14px; color:#64748b;")
        self.image3d_debug_threshold.setMinimumSize(280, 200)
        self.image3d_debug_threshold.setScaledContents(True)

        self.image3d_debug_combined = QLabel("Debug combined")
        self.image3d_debug_combined.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image3d_debug_combined.setStyleSheet("background:#0b1220; border-radius:14px; color:#64748b;")
        self.image3d_debug_combined.setMinimumSize(280, 200)
        self.image3d_debug_combined.setScaledContents(True)

        debug_lay.addWidget(card("Debug Threshold", self.image3d_debug_threshold), 1)
        debug_lay.addWidget(card("Debug Combined", self.image3d_debug_combined), 1)

        views.addWidget(card("Image 3D Live", self.image3d_label), 3)
        views.addWidget(debug_panel, 2)

        help_text = QLabel(
            "Workflow: start camera, show the chessboard, capture 12+ views, calibrate, then place an object on the board. "
            "Width and length are measured on the board plane in mm. Height is shown as an estimate only."
        )
        help_text.setWordWrap(True)
        help_text.setStyleSheet("color:#94a3b8;")

        lay.addWidget(controls)
        lay.addLayout(views, 1)
        lay.addWidget(help_text)
        return page

    def goto_measure(self):
        self.pages.setCurrentIndex(3)
        self.lbl_title.setText("Mesure d'objet")
        self.lbl_sub.setText("Mesure avancée : calibration, contours, surface")
        self._set_nav_checked("btn_measure")

    def goto_image_3d(self):
        self.pages.setCurrentIndex(4)
        self.lbl_title.setText("Image 3D")
        self.lbl_sub.setText("Calibration par damier + mesure en mm sur un plan de référence")
        self._set_nav_checked("btn_image_3d")

    def toggle_image3d_camera(self):
        if self.image3d_running:
            self.stop_image3d_camera()
        else:
            self.start_image3d_camera()

    def start_image3d_camera(self):
        self._stop_other_camera_modes()
        self.image3d_cap = self._open_cv_camera_capture(self.cfg.cam_index)
        if not self.image3d_cap.isOpened():
            self.image3d_label.setText("Camera not found")
            self.image3d_status.setText("Unable to open camera for Image 3D.")
            self.statusBar().showMessage("Image 3D: unable to open camera (camera busy or unavailable)")
            return

        self.image3d_running = True
        self.image3d_btn_start.setText("Stop Image 3D")
        self.image3d_timer = QTimer()
        self.image3d_timer.timeout.connect(self.image3d_tick)
        self.image3d_timer.start(30)
        self.image3d_status.setText("Camera running. Show the chessboard to calibrate or measure.")
        self._update_image3d_status_labels()

    def stop_image3d_camera(self):
        self.image3d_running = False
        self.image3d_btn_start.setText("Start Image 3D")
        if self.image3d_timer is not None:
            self.image3d_timer.stop()
            self.image3d_timer = None
        if self.image3d_cap is not None:
            self.image3d_cap.release()
            self.image3d_cap = None
        self.image3d_label.setText("Image 3D stopped")
        self.image3d_debug_threshold.setText("Debug threshold")
        self.image3d_debug_combined.setText("Debug combined")

    def capture_image3d_sample(self):
        if not self.image3d_running or self.image3d_cap is None:
            self.image3d_status.setText("Start Image 3D first, then capture chessboard samples.")
            return
        ret, frame = self.image3d_cap.read()
        if not ret or frame is None:
            self.image3d_status.setText("Failed to read a frame for calibration capture.")
            return
        success, message = self.image3d_backend.capture_calibration_sample(frame)
        self.image3d_status.setText(message)
        self._update_image3d_status_labels()
        if success:
            self.statusBar().showMessage(message)

    def run_image3d_calibration(self):
        success, message = self.image3d_backend.calibrate()
        self.image3d_status.setText(message)
        self._update_image3d_status_labels()
        self.statusBar().showMessage(message)

    def reset_image3d_samples(self):
        self.image3d_backend.reset_calibration_samples()
        self.image3d_status.setText("Captured Image 3D calibration samples cleared.")
        self._update_image3d_status_labels()

    def delete_image3d_calibration(self):
        self.image3d_backend.delete_saved_calibration()
        self.image3d_status.setText("Saved Image 3D calibration deleted.")
        self._update_image3d_status_labels()

    def _pixmap_from_gray(self, image_gray):
        if image_gray is None:
            return None
        if len(image_gray.shape) == 2:
            rgb = cv2.cvtColor(image_gray, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(image_gray, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        return QPixmap.fromImage(qimg)

    def _update_image3d_status_labels(self):
        rms_text = f"{self.image3d_backend.rms_error:.4f} px" if self.image3d_backend.rms_error is not None else "N/A"
        self.image3d_results.setText(
            "Width: N/A\n"
            "Length: N/A\n"
            "Height: N/A\n"
            f"RMS: {rms_text}\n"
            f"Samples: {len(self.image3d_backend.image_points)}"
        )

    def image3d_tick(self):
        if self.image3d_cap is None or not self.image3d_cap.isOpened():
            self.stop_image3d_camera()
            return

        ret, frame = self.image3d_cap.read()
        if not ret or frame is None:
            self.image3d_label.setText("No frame")
            return

        debug_enabled = self.image3d_chk_debug.isChecked()
        state, debug_images = self.image3d_backend.process_frame(frame, debug=debug_enabled)
        self.image3d_label.setPixmap(frame_to_pixmap(state.frame_bgr))

        width_text = f"{state.width_mm:.1f} mm" if state.width_mm is not None else "N/A"
        length_text = f"{state.length_mm:.1f} mm" if state.length_mm is not None else "N/A"
        height_text = f"~{state.estimated_height_mm:.1f} mm" if state.estimated_height_mm is not None else "N/A (need calibration)"
        rms_text = f"{state.rms_error:.4f} px" if state.rms_error is not None else "N/A"
        self.image3d_results.setText(
            f"Width: {width_text}\n"
            f"Length: {length_text}\n"
            f"Height: {height_text}\n"
            f"RMS: {rms_text}\n"
            f"Samples: {state.calibration_samples}"
        )
        self.image3d_status.setText(state.message)

        if debug_enabled and debug_images:
            threshold_pix = self._pixmap_from_gray(debug_images.get("threshold"))
            combined_pix = self._pixmap_from_gray(debug_images.get("combined"))
            if threshold_pix is not None:
                self.image3d_debug_threshold.setPixmap(threshold_pix)
            if combined_pix is not None:
                self.image3d_debug_combined.setPixmap(combined_pix)
        else:
            self.image3d_debug_threshold.setText("Enable debug processing")
            self.image3d_debug_combined.setText("Enable debug processing")

    def _build_object_detection_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        top = QFrame()
        top.setObjectName("Card")
        top_lay = QHBoxLayout(top)
        top_lay.setContentsMargins(16, 16, 16, 16)
        top_lay.setSpacing(12)

        self.object_search = QLineEdit()
        self.object_search.setPlaceholderText("Enter any object name, e.g. cell phone, bottle, person")
        self.object_search.setText(self.target_object)
        self.object_search.returnPressed.connect(self.apply_object_detection_target)
        self.object_search_completer = QCompleter(self._get_object_list(), self)
        self.object_search_completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.object_search_completer.setFilterMode(Qt.MatchFlag.MatchContains)
        self.object_search_completer.setCompletionMode(QCompleter.CompletionMode.PopupCompletion)
        self.object_search.setCompleter(self.object_search_completer)

        self.object_confidence_spin = QDoubleSpinBox()
        self.object_confidence_spin.setRange(0.01, 1.0)
        self.object_confidence_spin.setSingleStep(0.05)
        self.object_confidence_spin.setDecimals(2)
        self.object_confidence_spin.setValue(self.cfg.object_confidence)
        self.object_confidence_spin.setSuffix(" conf")

        self.btn_object_apply = QPushButton("Apply")
        self.btn_object_apply.setObjectName("SecondaryBtn")
        self.btn_object_apply.clicked.connect(self.apply_object_detection_target)

        self.btn_object_start = QPushButton("Start Object Detection")
        self.btn_object_start.clicked.connect(self.toggle_object_detection)

        top_lay.addWidget(QLabel("Search object:"))
        top_lay.addWidget(self.object_search, 1)
        top_lay.addWidget(QLabel("Confidence:"))
        top_lay.addWidget(self.object_confidence_spin)
        top_lay.addWidget(self.btn_object_apply)
        top_lay.addWidget(self.btn_object_start)

        self.object_detection_label = QLabel("Object detection stopped")
        self.object_detection_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.object_detection_label.setStyleSheet("background:#0b1220; border-radius:14px; color:#e5e7eb;")
        self.object_detection_label.setMinimumSize(520, 380)
        self.object_detection_label.setScaledContents(True)

        self.object_detection_status = QLabel(f"Watching for: {self.target_object}")
        self.object_detection_status.setStyleSheet("color:#94a3b8;")

        lay.addWidget(top)
        lay.addWidget(card("Object Detection", self.object_detection_label), 1)
        lay.addWidget(self.object_detection_status)
        return page

    def goto_object_detection(self):
        self.pages.setCurrentIndex(5)
        self.lbl_title.setText("Object Detection")
        self.lbl_sub.setText("YOLO object detection with custom search and Arduino buzzer")
        self._set_nav_checked("btn_object_detection")

    def apply_object_detection_target(self):
        target = self.object_search.text().strip().lower()
        if not target:
            target = "cell phone"
            self.object_search.setText(target)
        confidence = float(self.object_confidence_spin.value())
        self.target_object = target
        self.cfg.target_object = target
        self.cfg.object_confidence = confidence
        self.object_detection_status.setText(f"Watching for: {self.target_object} | confidence >= {confidence:.2f}")
        self.statusBar().showMessage(f"Target object set to: {self.target_object} | confidence: {confidence:.2f}")

    def toggle_object_detection(self):
        if self.object_detection_running:
            self.stop_object_detection()
        else:
            self.start_object_detection()

    def start_object_detection(self):
        self._stop_other_camera_modes()
        self.apply_object_detection_target()
        if self.yolo is None:
            self.object_detection_label.setText("YOLO model unavailable")
            self.statusBar().showMessage("Unable to start object detection: YOLO model unavailable")
            return

        self.object_detection_cap = self._open_cv_camera_capture(self.cfg.cam_index)
        if not self.object_detection_cap.isOpened():
            self.object_detection_label.setText("Camera not found")
            self.statusBar().showMessage("Unable to start object detection: camera not found")
            return

        self.object_detection_running = True
        self.btn_object_start.setText("Stop Object Detection")
        self.last_buzz_time = 0.0
        self.object_detection_timer = QTimer()
        self.object_detection_timer.timeout.connect(self.object_detection_tick)
        self.object_detection_timer.start(30)
        self.object_detection_status.setText(
            f"Watching for: {self.target_object} | confidence >= {self.cfg.object_confidence:.2f}"
        )

    def stop_object_detection(self):
        self.object_detection_running = False
        self.btn_object_start.setText("Start Object Detection")
        if self.object_detection_timer is not None:
            self.object_detection_timer.stop()
            self.object_detection_timer = None
        if self.object_detection_cap is not None:
            self.object_detection_cap.release()
            self.object_detection_cap = None
        self.object_detection_label.setText("Object detection stopped")
        self.object_detection_status.setText(
            f"Watching for: {self.target_object} | confidence >= {self.cfg.object_confidence:.2f}"
        )

    def object_detection_tick(self):
        if self.object_detection_cap is None or not self.object_detection_cap.isOpened():
            self.stop_object_detection()
            return

        ret, frame = self.object_detection_cap.read()
        if not ret:
            self.object_detection_label.setText("No frame")
            return

        results = self.yolo(frame, verbose=False, conf=self.cfg.object_confidence)
        detections = results[0]
        detected = False

        for box in detections.boxes:
            class_id = int(box.cls[0])
            label = str(self.yolo.names[class_id]).lower()
            if label == self.target_object:
                detected = True
                break

        now = time.time()
        if detected and (now - self.last_buzz_time) > self.buzzer_cooldown:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b'1')
                except Exception:
                    pass
            self.last_buzz_time = now

        annotated_frame = detections.plot()
        if detected:
            status = f"DETECTED: {self.target_object} | confidence >= {self.cfg.object_confidence:.2f}"
        else:
            status = f"Watching for: {self.target_object} | confidence >= {self.cfg.object_confidence:.2f}"
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(annotated_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg)
        self.object_detection_label.setPixmap(pix)
        self.object_detection_status.setText(status)

    def toggle_measure_camera(self):
        if not hasattr(self, 'measure_running'):
            self.measure_running = False
        if not self.measure_running:
            self.start_measure_camera()
        else:
            self.stop_measure_camera()

    def start_measure_camera(self):
        self._stop_other_camera_modes()
        self.measure_cap = self._open_cv_camera_capture(self.cfg.cam_index)
        if not self.measure_cap.isOpened():
            self.measure_label.setText("Camera not found")
            return
        self.measure_running = True
        self.btn_measure_start.setText("Stop Mesure d'objet")
        self.measure_timer = QTimer()
        self.measure_timer.timeout.connect(self.measure_tick)
        self.measure_timer.start(30)

        # Variables pour calibration et mesure avancée
        self.measure_calibrated = False
        self.measure_pixels_per_mm = None
        self.measure_ref_points = []  # Pour calibration (clics)
        self.measure_contour = None

        self.measure_label.mousePressEvent = self.measure_mouse_press

    def stop_measure_camera(self):
        self.measure_running = False
        self.btn_measure_start.setText("Start Mesure d'objet")
        if hasattr(self, 'measure_timer'):
            self.measure_timer.stop()
        if hasattr(self, 'measure_cap'):
            self.measure_cap.release()
        self.measure_label.setText("Camera stopped")

    def measure_mouse_press(self, event):
        # Calibration : enregistrer 2 points de référence (clics)
        if not self.measure_calibrated and self.measure_running:
            x = event.position().x()
            y = event.position().y()
            self.measure_ref_points.append((int(x), int(y)))
            if len(self.measure_ref_points) == 2:
                # Demander à l'utilisateur la distance réelle (mm)
                from PyQt6.QtWidgets import QInputDialog
                dist, ok = QInputDialog.getDouble(self, "Calibration", "Distance réelle entre les 2 points (mm) :", 10, 0.1, 1000, 2)
                if ok:
                    dx = self.measure_ref_points[0][0] - self.measure_ref_points[1][0]
                    dy = self.measure_ref_points[0][1] - self.measure_ref_points[1][1]
                    px_dist = (dx**2 + dy**2) ** 0.5
                    self.measure_pixels_per_mm = px_dist / dist
                    self.measure_calibrated = True
        # Pourrait aussi servir à sélectionner un objet à mesurer

    def measure_tick(self):
        if not hasattr(self, 'measure_cap') or not self.measure_cap.isOpened():
            self.stop_measure_camera()
            return
        ret, frame = self.measure_cap.read()
        if not ret:
            self.measure_label.setText("No frame")
            return
        display = frame.copy()
        h, w, _ = display.shape

        # Afficher les points de calibration
        for pt in self.measure_ref_points:
            cv2.circle(display, pt, 6, (0,255,255), -1)
        if len(self.measure_ref_points) == 2:
            cv2.line(display, self.measure_ref_points[0], self.measure_ref_points[1], (0,255,255), 2)

        # Détection avancée : trouver le plus grand contour fermé (objet)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edged = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Valeurs par défaut
        area_mm = None
        length_mm = None
        width_mm = None

        if contours:
            c = max(contours, key=cv2.contourArea)
            area_px = cv2.contourArea(c)
            cv2.drawContours(display, [c], -1, (0,0,255), 2)
            # Bounding box
            x, y, w_box, h_box = cv2.boundingRect(c)
            cv2.rectangle(display, (x, y), (x + w_box, y + h_box), (255, 0, 0), 2)
            if self.measure_calibrated and self.measure_pixels_per_mm:
                area_mm = area_px / (self.measure_pixels_per_mm ** 2)
                length_mm = w_box / self.measure_pixels_per_mm
                width_mm = h_box / self.measure_pixels_per_mm

        # Affichage des mesures ou aide
        if not self.measure_calibrated or not self.measure_pixels_per_mm:
            cv2.putText(display, "Calibration: cliquez sur 2 points", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,140,255), 2)
        else:
            surf_txt = f"Surface: {area_mm:.2f} mm2" if area_mm is not None else "Surface: N/A"
            long_txt = f"Longueur: {length_mm:.2f} mm" if length_mm is not None else "Longueur: N/A"
            larg_txt = f"Largeur: {width_mm:.2f} mm" if width_mm is not None else "Largeur: N/A"
            cv2.putText(display, surf_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.putText(display, long_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            cv2.putText(display, larg_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        # Affichage image
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg)
        self.measure_label.setPixmap(pix)

    def _build_face_tracking_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.face_tracking_label = QLabel("Camera stopped")
        self.face_tracking_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.face_tracking_label.setStyleSheet("background:#0b1220; border-radius:14px; color:#e5e7eb;")
        self.face_tracking_label.setMinimumSize(520, 380)
        self.face_tracking_label.setScaledContents(True)

        self.btn_face_tracking_start = QPushButton("Start Face Tracking")
        self.btn_face_tracking_start.setObjectName("SecondaryBtn")
        self.btn_face_tracking_start.clicked.connect(self.toggle_face_tracking_camera)

        lay.addWidget(card("Face Tracking (Crosshair)", self.face_tracking_label), 1)
        lay.addWidget(self.btn_face_tracking_start)
        return page

    def toggle_face_tracking_camera(self):
        if not hasattr(self, 'face_tracking_running'):
            self.face_tracking_running = False
        if not self.face_tracking_running:
            self.start_face_tracking_camera()
        else:
            self.stop_face_tracking_camera()

    def start_face_tracking_camera(self):
        self._stop_other_camera_modes()
        self.face_tracking_cap = cv2.VideoCapture(self.cfg.cam_index)
        if not self.face_tracking_cap.isOpened():
            self.face_tracking_label.setText("Camera not found")
            return
        self.face_tracking_running = True
        self.btn_face_tracking_start.setText("Stop Face Tracking")
        self.face_tracking_timer = QTimer()
        self.face_tracking_timer.timeout.connect(self.face_tracking_tick)
        self.face_tracking_timer.start(30)

    def stop_face_tracking_camera(self):
        self.face_tracking_running = False
        self.btn_face_tracking_start.setText("Start Face Tracking")
        if hasattr(self, 'face_tracking_timer'):
            self.face_tracking_timer.stop()
        if hasattr(self, 'face_tracking_cap'):
            self.face_tracking_cap.release()
        self.face_tracking_label.setText("Camera stopped")

    def face_tracking_tick(self):
        if not hasattr(self, 'face_tracking_cap') or not self.face_tracking_cap.isOpened():
            self.stop_face_tracking_camera()
            return
        ret, frame = self.face_tracking_cap.read()
        if not ret:
            self.face_tracking_label.setText("No frame")
            return
        # Détection visage + crosshair
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.2, 5)
        h, w, _ = frame.shape
        # Dessiner crosshair au centre du visage détecté (ou au centre si rien)
        if len(faces) > 0:
            (x, y, fw, fh) = faces[0]
            cx = x + fw // 2
            cy = y + fh // 2
        else:
            cx = w // 2
            cy = h // 2
        # Dessiner crosshair
        color = (0, 0, 255) if len(faces) > 0 else (100, 100, 100)
        cv2.drawMarker(frame, (cx, cy), color, markerType=cv2.MARKER_CROSS, markerSize=40, thickness=2)
        # Dessiner axes X et Y
        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)  # Axe Y (vertical)
        cv2.line(frame, (0, cy), (w, cy), (0, 255, 255), 1)  # Axe X (horizontal)
        # Optionnel: dessiner rectangle visage
        for (x, y, fw, fh) in faces:
            cv2.rectangle(frame, (x, y), (x+fw, y+fh), (0,255,0), 2)
        # Affichage
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg)
        self.face_tracking_label.setPixmap(pix)

    def goto_face_tracking(self):
        self.pages.setCurrentIndex(2)
        self.lbl_title.setText("Face Tracking")
        self.lbl_sub.setText("Détection visage + crosshair (OpenCV)")
        self._set_nav_checked("btn_face_tracking")

    # ---------- TOPBAR ----------
    def _build_topbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("TopBar")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(10)

        left = QVBoxLayout()
        self.lbl_title = QLabel("Home")
        self.lbl_title.setObjectName("AppTitle")
        self.lbl_sub = QLabel("Menu d’entrée • Choisis une section à gauche")
        self.lbl_sub.setObjectName("AppSubTitle")
        left.addWidget(self.lbl_title)
        left.addWidget(self.lbl_sub)
        lay.addLayout(left)

        lay.addStretch()

        self.pill_conn = QLabel("Disconnected")
        self.pill_conn.setObjectName("Pill")
        lay.addWidget(self.pill_conn)

        self.port_combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setObjectName("SecondaryBtn")
        self.btn_refresh.clicked.connect(self.refresh_ports)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)

        self.chk_auto_cam = QCheckBox("Auto camera (2s)")
        self.chk_detect = QCheckBox("OpenCV detect")

        self.btn_camera = QPushButton("Start camera")
        self.btn_camera.clicked.connect(self.toggle_camera)

        # NEW: Screenshot button
        self.btn_shot = QPushButton("Screenshot")
        self.btn_shot.setObjectName("SecondaryBtn")
        self.btn_shot.clicked.connect(self.take_screenshot_auto)
        self.btn_shot.setToolTip("Save screenshot to /python/screenshots (Ctrl+Shift+S)")

        lay.addWidget(self.port_combo)
        lay.addWidget(self.btn_refresh)
        lay.addWidget(self.btn_connect)
        lay.addSpacing(10)
        lay.addWidget(self.chk_auto_cam)
        lay.addWidget(self.chk_detect)
        lay.addWidget(self.btn_camera)
        lay.addWidget(self.btn_shot)

        return bar

    # ---------- PAGES ----------
    def _build_home_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        hero = QFrame()
        hero.setObjectName("Card")
        h = QVBoxLayout(hero)
        h.setContentsMargins(18, 18, 18, 18)
        h.setSpacing(10)

        t = QLabel("Bienvenue 👋")
        t.setObjectName("CardTitle")
        h.addWidget(t)

        msg = QLabel(
            "Pilote rapidement les modules principaux depuis Home.\n"
            "• Navigation directe vers chaque fonctionnalite\n"
            "• Actions systeme (serie, camera, capture, configuration)\n"
            "• Etat temps reel radar/camera et derniers screenshots"
        )
        msg.setStyleSheet("color:#94a3b8;")
        msg.setWordWrap(True)
        h.addWidget(msg)

        btn_row = QHBoxLayout()
        b1 = QPushButton("Open Live")
        b1.clicked.connect(self.goto_live)
        b2 = QPushButton("Open Object Detection")
        b2.clicked.connect(self.goto_object_detection)
        b3 = QPushButton("Open Settings")
        b3.setObjectName("SecondaryBtn")
        b3.clicked.connect(self.goto_settings)
        btn_row.addWidget(b1)
        btn_row.addWidget(b2)
        btn_row.addWidget(b3)
        btn_row.addStretch()
        h.addLayout(btn_row)

        quick_nav = QFrame()
        quick_nav.setObjectName("Card")
        qn = QGridLayout(quick_nav)
        qn.setContentsMargins(16, 16, 16, 16)
        qn.setHorizontalSpacing(10)
        qn.setVerticalSpacing(10)

        nav_buttons = [
            ("Live", self.goto_live),
            ("Face Tracking", self.goto_face_tracking),
            ("Mesure d'objet", self.goto_measure),
            ("Image 3D", self.goto_image_3d),
            ("Object Detection", self.goto_object_detection),
            ("Settings", self.goto_settings),
        ]
        for idx, (label, callback) in enumerate(nav_buttons):
            btn = QPushButton(label)
            btn.setObjectName("SecondaryBtn")
            btn.clicked.connect(callback)
            qn.addWidget(btn, idx // 3, idx % 3)

        system = QFrame()
        system.setObjectName("Card")
        sys_lay = QVBoxLayout(system)
        sys_lay.setContentsMargins(16, 16, 16, 16)
        sys_lay.setSpacing(10)

        system_title = QLabel("Actions rapides")
        system_title.setObjectName("CardTitle")
        sys_lay.addWidget(system_title)

        sys_row1 = QHBoxLayout()
        self.home_btn_connect = QPushButton("Connect serial")
        self.home_btn_connect.setObjectName("SecondaryBtn")
        self.home_btn_connect.clicked.connect(self.toggle_connect)

        self.home_btn_camera = QPushButton("Start camera")
        self.home_btn_camera.clicked.connect(self.toggle_camera)

        self.home_btn_shot = QPushButton("Screenshot")
        self.home_btn_shot.setObjectName("SecondaryBtn")
        self.home_btn_shot.clicked.connect(self.take_screenshot_auto)

        sys_row1.addWidget(self.home_btn_connect)
        sys_row1.addWidget(self.home_btn_camera)
        sys_row1.addWidget(self.home_btn_shot)
        sys_lay.addLayout(sys_row1)

        sys_row2 = QHBoxLayout()
        self.home_btn_save_cfg = QPushButton("Save config")
        self.home_btn_save_cfg.setObjectName("SecondaryBtn")
        self.home_btn_save_cfg.clicked.connect(self.save_config)

        self.home_btn_open_cfg = QPushButton("Open config")
        self.home_btn_open_cfg.setObjectName("SecondaryBtn")
        self.home_btn_open_cfg.clicked.connect(self.open_config_dialog)

        self.home_btn_refresh_shots = QPushButton("Refresh screenshots")
        self.home_btn_refresh_shots.setObjectName("SecondaryBtn")
        self.home_btn_refresh_shots.clicked.connect(self._refresh_home_dashboard)

        sys_row2.addWidget(self.home_btn_save_cfg)
        sys_row2.addWidget(self.home_btn_open_cfg)
        sys_row2.addWidget(self.home_btn_refresh_shots)
        sys_lay.addLayout(sys_row2)

        status = QFrame()
        status.setObjectName("Card")
        st = QGridLayout(status)
        st.setContentsMargins(16, 16, 16, 16)
        st.setHorizontalSpacing(16)
        st.setVerticalSpacing(8)

        status_title = QLabel("Statut du systeme")
        status_title.setObjectName("CardTitle")
        st.addWidget(status_title, 0, 0, 1, 2)

        self.home_stat_serial = QLabel("Serial: --")
        self.home_stat_camera = QLabel("Camera: --")
        self.home_stat_radar = QLabel("Radar state: --")
        self.home_stat_distance = QLabel("Distance: --")
        self.home_stat_auto = QLabel("Auto camera: --")
        self.home_stat_detect = QLabel("OpenCV detect: --")
        self.home_stat_target = QLabel("Target: --")
        self.home_stat_shots = QLabel("Screenshots: --")

        stats = [
            self.home_stat_serial,
            self.home_stat_camera,
            self.home_stat_radar,
            self.home_stat_distance,
            self.home_stat_auto,
            self.home_stat_detect,
            self.home_stat_target,
            self.home_stat_shots,
        ]
        for i, lbl in enumerate(stats):
            lbl.setStyleSheet("color:#cbd5e1;")
            st.addWidget(lbl, 1 + (i // 2), i % 2)

        recent = QFrame()
        recent.setObjectName("Card")
        rc = QVBoxLayout(recent)
        rc.setContentsMargins(16, 16, 16, 16)
        rc.setSpacing(8)

        rt = QLabel("Derniers screenshots")
        rt.setObjectName("CardTitle")
        rc.addWidget(rt)

        self.home_shot_1 = QLabel("Aucun screenshot")
        self.home_shot_2 = QLabel("-")
        self.home_shot_3 = QLabel("-")

        for lbl in [self.home_shot_1, self.home_shot_2, self.home_shot_3]:
            lbl.setStyleSheet("color:#94a3b8;")
            rc.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(12)
        row.addWidget(quick_nav, 2)
        row.addWidget(system, 2)

        bottom = QHBoxLayout()
        bottom.setSpacing(12)
        bottom.addWidget(status, 3)
        bottom.addWidget(recent, 2)

        lay.addWidget(hero)
        lay.addLayout(row)
        lay.addLayout(bottom)
        lay.addStretch()
        return page

    def _refresh_home_dashboard(self):
        if not hasattr(self, "home_stat_serial"):
            return

        serial_ok = self.ser is not None and bool(getattr(self.ser, "is_open", True))
        cam_ok = self.cam is not None and self.camera_thread is not None and self.camera_thread.isRunning()
        auto_mode = bool(self.chk_auto_cam.isChecked()) if hasattr(self, "chk_auto_cam") else bool(self.cfg.auto_camera_from_radar)
        detect_mode = bool(self.chk_detect.isChecked()) if hasattr(self, "chk_detect") else bool(self.cfg.opencv_detect)

        self.home_stat_serial.setText(f"Serial: {'Connected' if serial_ok else 'Disconnected'}")
        self.home_stat_camera.setText(f"Camera: {'Running' if cam_ok else 'Stopped'}")
        self.home_stat_radar.setText(f"Radar state: {str(self.state).upper()}")
        if self.last_distance is None:
            self.home_stat_distance.setText("Distance: N/A")
        else:
            self.home_stat_distance.setText(f"Distance: {self.last_distance:.1f} cm")
        self.home_stat_auto.setText(f"Auto camera: {'ON' if auto_mode else 'OFF'}")
        self.home_stat_detect.setText(f"OpenCV detect: {'ON' if detect_mode else 'OFF'}")
        self.home_stat_target.setText(f"Target: {self.target_object}")

        if hasattr(self, "home_btn_connect"):
            self.home_btn_connect.setText("Disconnect serial" if serial_ok else "Connect serial")
        if hasattr(self, "home_btn_camera"):
            self.home_btn_camera.setText("Stop camera" if cam_ok else "Start camera")

        self._refresh_home_screenshots()

    def _refresh_home_screenshots(self):
        if not hasattr(self, "home_shot_1"):
            return

        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)
        files = sorted(
            SCREENSHOT_DIR.glob("ui_*.png"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )

        self.home_stat_shots.setText(f"Screenshots: {len(files)}")

        labels = [self.home_shot_1, self.home_shot_2, self.home_shot_3]
        for i, lbl in enumerate(labels):
            if i < len(files):
                lbl.setText(files[i].name)
            elif i == 0:
                lbl.setText("Aucun screenshot")
            else:
                lbl.setText("-")

    def _build_live_page(self) -> QWidget:
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.radar_widget = RadarWidget(self.cfg)

        self.cam_label = QLabel("Camera stopped")
        self.cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cam_label.setStyleSheet("background:#0b1220; border-radius:14px; color:#e5e7eb;")
        self.cam_label.setMinimumSize(520, 380)
        self.cam_label.setScaledContents(True)

        lay.addWidget(card("Radar", self.radar_widget), 1)
        lay.addWidget(card("Camera", self.cam_label), 1)
        return page

    def _build_settings_page(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(12)

        row = QHBoxLayout()
        row.setSpacing(12)

        logic = QWidget()
        gl = QGridLayout(logic)
        gl.setContentsMargins(0, 0, 0, 0)
        gl.setHorizontalSpacing(12)
        gl.setVerticalSpacing(10)

        self.s_on = QDoubleSpinBox(); self.s_on.setRange(1, 400); self.s_on.setSuffix(" cm")
        self.s_off = QDoubleSpinBox(); self.s_off.setRange(1, 400); self.s_off.setSuffix(" cm")
        self.s_trigger = QDoubleSpinBox(); self.s_trigger.setRange(0.1, 10.0); self.s_trigger.setSingleStep(0.1); self.s_trigger.setSuffix(" s")
        self.s_close = QDoubleSpinBox(); self.s_close.setRange(0.1, 60.0); self.s_close.setSingleStep(0.5); self.s_close.setSuffix(" s")

        gl.addWidget(QLabel("ON threshold"), 0, 0); gl.addWidget(self.s_on, 0, 1)
        gl.addWidget(QLabel("OFF threshold"), 1, 0); gl.addWidget(self.s_off, 1, 1)
        gl.addWidget(QLabel("Confirm time"), 2, 0); gl.addWidget(self.s_trigger, 2, 1)
        gl.addWidget(QLabel("Close after loss"), 3, 0); gl.addWidget(self.s_close, 3, 1)

        row.addWidget(card("Automation", logic), 1)

        calib = QWidget()
        cg = QGridLayout(calib)
        cg.setContentsMargins(0, 0, 0, 0)
        cg.setHorizontalSpacing(12)
        cg.setVerticalSpacing(10)

        self.s_servo_min = QDoubleSpinBox(); self.s_servo_min.setRange(0, 180); self.s_servo_min.setSuffix("°")
        self.s_servo_max = QDoubleSpinBox(); self.s_servo_max.setRange(0, 180); self.s_servo_max.setSuffix("°")
        self.s_offset = QDoubleSpinBox(); self.s_offset.setRange(-90, 90); self.s_offset.setSingleStep(1); self.s_offset.setSuffix("°")
        self.chk_flip = QCheckBox("Flip left/right")
        self.s_range = QDoubleSpinBox(); self.s_range.setRange(10, 500); self.s_range.setSuffix(" cm")

        cg.addWidget(QLabel("Servo min angle"), 0, 0); cg.addWidget(self.s_servo_min, 0, 1)
        cg.addWidget(QLabel("Servo max angle"), 1, 0); cg.addWidget(self.s_servo_max, 1, 1)
        cg.addWidget(QLabel("Radar offset"), 2, 0); cg.addWidget(self.s_offset, 2, 1)
        cg.addWidget(QLabel("Max range"), 3, 0); cg.addWidget(self.s_range, 3, 1)
        cg.addWidget(self.chk_flip, 4, 0, 1, 2)

        row.addWidget(card("Radar Calibration", calib), 1)
        outer.addLayout(row)

        row2 = QHBoxLayout()
        row2.setSpacing(12)

        ocv = QWidget()
        og = QGridLayout(ocv)
        og.setContentsMargins(0, 0, 0, 0)
        og.setHorizontalSpacing(12)
        og.setVerticalSpacing(10)

        self.s_hold = QDoubleSpinBox(); self.s_hold.setRange(0.1, 10.0); self.s_hold.setSingleStep(0.1); self.s_hold.setSuffix(" s")
        self.s_every = QSpinBox(); self.s_every.setRange(1, 10)
        self.s_cam_index = QSpinBox(); self.s_cam_index.setRange(0, 10)

        # Target object selection
        self.cmb_target = QComboBox()
        self.cmb_target.setEditable(True)
        self.cmb_target.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        objects = sorted(set(self._get_object_list()))
        self.cmb_target.addItems(objects)
        self.cmb_target.setToolTip("Enter or select the object class to detect (e.g. cell phone)")

        og.addWidget(QLabel("Target object"), 0, 0); og.addWidget(self.cmb_target, 0, 1)
        og.addWidget(QLabel("Hold for 'CIBLE'"), 1, 0); og.addWidget(self.s_hold, 1, 1)
        og.addWidget(QLabel("Detect every N frames"), 2, 0); og.addWidget(self.s_every, 2, 1)
        og.addWidget(QLabel("Camera index"), 3, 0); og.addWidget(self.s_cam_index, 3, 1)

        row2.addWidget(card("OpenCV", ocv), 1)

        image3d = QWidget()
        ig = QGridLayout(image3d)
        ig.setContentsMargins(0, 0, 0, 0)
        ig.setHorizontalSpacing(12)
        ig.setVerticalSpacing(10)

        self.s_image3d_cols = QSpinBox(); self.s_image3d_cols.setRange(3, 20)
        self.s_image3d_rows = QSpinBox(); self.s_image3d_rows.setRange(3, 20)
        self.s_image3d_square = QDoubleSpinBox(); self.s_image3d_square.setRange(1.0, 100.0); self.s_image3d_square.setDecimals(2); self.s_image3d_square.setSuffix(" mm")
        self.s_image3d_samples = QSpinBox(); self.s_image3d_samples.setRange(4, 50)
        self.s_image3d_blur = QSpinBox(); self.s_image3d_blur.setRange(3, 31); self.s_image3d_blur.setSingleStep(2)
        self.s_image3d_area = QSpinBox(); self.s_image3d_area.setRange(100, 500000)
        self.chk_image3d_debug = QCheckBox("Image 3D debug by default")

        ig.addWidget(QLabel("Chessboard cols"), 0, 0); ig.addWidget(self.s_image3d_cols, 0, 1)
        ig.addWidget(QLabel("Chessboard rows"), 1, 0); ig.addWidget(self.s_image3d_rows, 1, 1)
        ig.addWidget(QLabel("Square size"), 2, 0); ig.addWidget(self.s_image3d_square, 2, 1)
        ig.addWidget(QLabel("Min calib views"), 3, 0); ig.addWidget(self.s_image3d_samples, 3, 1)
        ig.addWidget(QLabel("Blur kernel"), 4, 0); ig.addWidget(self.s_image3d_blur, 4, 1)
        ig.addWidget(QLabel("Min contour area"), 5, 0); ig.addWidget(self.s_image3d_area, 5, 1)
        ig.addWidget(self.chk_image3d_debug, 6, 0, 1, 2)

        row2.addWidget(card("Image 3D", image3d), 1)

        actions = QWidget()
        al = QVBoxLayout(actions)
        al.setContentsMargins(0, 0, 0, 0)
        al.setSpacing(10)

        self.btn_apply = QPushButton("Apply changes")
        self.btn_apply.clicked.connect(self.apply_settings)

        self.btn_save = QPushButton("Save config")
        self.btn_save.setObjectName("SecondaryBtn")
        self.btn_save.clicked.connect(self.save_config)

        self.btn_load = QPushButton("Open config...")
        self.btn_load.setObjectName("SecondaryBtn")
        self.btn_load.clicked.connect(self.open_config_dialog)

        self.btn_shot_settings = QPushButton("Screenshot now")
        self.btn_shot_settings.setObjectName("SecondaryBtn")
        self.btn_shot_settings.clicked.connect(self.take_screenshot_auto)

        tip = QLabel("Screenshot = preuve visuelle.\nFichiers dans python/screenshots/")
        tip.setStyleSheet("color:#94a3b8;")

        al.addWidget(self.btn_apply)
        al.addWidget(self.btn_save)
        al.addWidget(self.btn_load)
        al.addWidget(self.btn_shot_settings)
        al.addStretch()
        al.addWidget(tip)

        row2.addWidget(card("Configuration", actions), 1)

        outer.addLayout(row2)
        outer.addStretch()
        return page

    def _build_about_page(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        about = QLabel(
            "Projet Suivi Cible\n\n"
            "• Radar Arduino: servo + HC-SR04\n"
            "• Desktop App: PyQt6\n"
            "• Vision: OpenCV (détection + overlay)\n\n"
            "Objectif: application stable, claire, et professionnelle."
        )
        about.setStyleSheet("color:#cbd5e1;")
        lay.addWidget(card("About", about))
        lay.addStretch()
        return page

    # ---------- NAV ----------
    def goto_home(self):
        self.pages.setCurrentIndex(0)
        self.lbl_title.setText("Home")
        self.lbl_sub.setText("Menu d’entrée • Choisis une section à gauche")
        self._set_nav_checked("btn_home")

    def goto_live(self):
        self.pages.setCurrentIndex(1)
        self.lbl_title.setText("Live")
        self.lbl_sub.setText("Radar en temps réel + caméra + OpenCV")
        self._set_nav_checked("btn_live")

    def goto_settings(self):
        self.pages.setCurrentIndex(6)
        self.lbl_title.setText("Settings")
        self.lbl_sub.setText("Seuils • calibration • performance • config")
        self._set_nav_checked("btn_settings")

    def goto_about(self):
        self.pages.setCurrentIndex(7)
        self.lbl_title.setText("About")
        self.lbl_sub.setText("Infos du projet")
        self._set_nav_checked("btn_about")

    # ---------- PORTS / SERIAL ----------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_com_ports()
        if not ports:
            self.port_combo.addItem("No ports", "")
            return
        for dev, desc in ports:
            self.port_combo.addItem(f"{dev} - {desc}", dev)

    def _set_connection_pill(self, connected: bool):
        if connected:
            self.pill_conn.setText("Connected")
            self.pill_conn.setObjectName("PillGreen")
        else:
            self.pill_conn.setText("Disconnected")
            self.pill_conn.setObjectName("PillRed")
        self.pill_conn.style().unpolish(self.pill_conn)
        self.pill_conn.style().polish(self.pill_conn)

    def toggle_connect(self):
        if self.ser is None:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Port", "Select a COM port.")
            return
        try:
            self.ser = serial.Serial(port, self.cfg.baud, timeout=self.cfg.serial_timeout_sec)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self.parser = SerialParser()
            self.btn_connect.setText("Disconnect")
            self._set_connection_pill(True)
            self.statusBar().showMessage(f"Connected to {port} @ {self.cfg.baud}")
        except PermissionError:
            QMessageBox.critical(self, "Serial", "Port busy. Close Serial Monitor / other scripts.")
        except Exception as e:
            QMessageBox.critical(self, "Serial", str(e))

    def disconnect_serial(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn_connect.setText("Connect")
        self._set_connection_pill(False)
        self.statusBar().showMessage("Disconnected")

        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history.clear()
        self.serial_buffer = ""
        self.last_real_angle_ts = 0.0
        self.synthetic_angle_raw = 90.0
        self.synthetic_dir = 1.0
        self.last_synth_update_ts = time.time()
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

    # ---------- CAMERA ----------
    def toggle_camera(self):
        if self.cam is None:
            self.open_camera()
        else:
            self.close_camera()

    def _stop_other_camera_modes(self):
        """Ensure only one OpenCV capture is active to avoid backend conflicts."""
        try:
            if self.cam is not None:
                self.close_camera()
        except Exception:
            pass
        try:
            if getattr(self, "object_detection_running", False):
                self.stop_object_detection()
        except Exception:
            pass
        try:
            if getattr(self, "measure_running", False):
                self.stop_measure_camera()
        except Exception:
            pass
        try:
            if getattr(self, "face_tracking_running", False):
                self.stop_face_tracking_camera()
        except Exception:
            pass
        try:
            if getattr(self, "image3d_running", False):
                self.stop_image3d_camera()
        except Exception:
            pass

    def _open_cv_camera_capture(self, cam_index: int):
        """Open a camera with backends that work reliably on Windows."""
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, None):
            cam = None
            try:
                if backend is None:
                    cam = cv2.VideoCapture(int(cam_index))
                else:
                    cam = cv2.VideoCapture(int(cam_index), backend)
                if cam is not None and cam.isOpened():
                    try:
                        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                    return cam
                if cam is not None:
                    cam.release()
            except Exception:
                try:
                    if cam is not None:
                        cam.release()
                except Exception:
                    pass
        return cv2.VideoCapture(int(cam_index))

    def _on_camera_error(self, message: str):
        self.statusBar().showMessage(message)
        try:
            QMessageBox.warning(self, "Camera", message)
        except Exception:
            pass
        self.close_camera()

    def _on_camera_frame(self, frame):
        # Keep only the latest frame to avoid queue buildup and UI lag.
        self.latest_frame = frame
        try:
            if getattr(self, "measure_running", False):
                self.stop_measure_camera()
        except Exception:
            pass
        try:
            if getattr(self, "face_tracking_running", False):
                self.stop_face_tracking_camera()
        except Exception:
            pass
        try:
            if getattr(self, "image3d_running", False):
                self.stop_image3d_camera()
        except Exception:
            pass

    def open_camera(self):
        self._stop_other_camera_modes()

        if self.camera_thread is not None and self.camera_thread.isRunning():
            return

        self.latest_frame = None
        self.camera_thread = CameraCaptureThread(int(self.cfg.cam_index))
        self.camera_thread.frame_ready.connect(self._on_camera_frame)
        self.camera_thread.camera_error.connect(self._on_camera_error)
        self.camera_thread.start()

        # Keep a non-None sentinel for existing logic paths that check self.cam.
        self.cam = True
        self.btn_camera.setText("Stop camera")
        self.target_start = None
        self.last_det_boxes = []
        self.last_det_ts = 0.0
        self.target_track.clear()
        self.predicted_points = []
        self.last_target_center = None


    def close_camera(self):
        if self.camera_thread is not None:
            try:
                self.camera_thread.stop()
                self.camera_thread.wait(1000)
            except Exception:
                pass
            self.camera_thread = None

        try:
            if self.cam:
                pass
        except Exception:
            pass
        self.cam = None
        self.latest_frame = None
        self.btn_camera.setText("Start camera")
        self.cam_label.setText("Camera stopped")
        self.target_start = None
        self.last_frame_bgr = None
        self.target_track.clear()
        self.predicted_points = []
        self.last_target_center = None

    def _get_target_center_from_boxes(self):
        """
        Retourne le centre (cx, cy) de la plus grande box correspondant
        à l'objet cible actuellement détecté.
        """
        target_boxes = [b for b in self.last_det_boxes if b[5]]
        if not target_boxes:
            return None

        best_box = max(target_boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        x1, y1, x2, y2, label, is_target = best_box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        return (cx, cy)

    def _update_trajectory(self, center, now):
        """
        Met à jour l'historique de trajectoire et prédit les prochains points.

        Formules:
            vx = dx / dt
            vy = dy / dt
            x_f = x + vx * t
            y_f = y + vy * t
        """
        self.predicted_points = []

        if center is None:
            return

        self.target_track.append((center[0], center[1], now))
        self.last_target_center = center

        if len(self.target_track) < 2:
            return

        x1, y1, t1 = self.target_track[-2]
        x2, y2, t2 = self.target_track[-1]

        dt = max(t2 - t1, self.cfg.trajectory_min_dt)
        vx = (x2 - x1) / dt
        vy = (y2 - y1) / dt

        max_speed_px_per_sec = 2000.0
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > max_speed_px_per_sec:
            return

        for i in range(1, self.cfg.trajectory_prediction_steps + 1):
            tf = i * self.cfg.trajectory_step_sec
            xf = int(x2 + vx * tf)
            yf = int(y2 + vy * tf)
            self.predicted_points.append((xf, yf))

    def _draw_trajectory_overlay(self, frame):
        """
        Dessine:
        - l'historique du mouvement
        - la flèche de direction
        - les points futurs prédits
        """
        if len(self.target_track) >= 2:
            pts = [(int(x), int(y)) for x, y, _ in self.target_track]
            for i in range(1, len(pts)):
                thickness = max(1, 4 - (len(pts) - i) // 3)
                cv2.line(frame, pts[i - 1], pts[i], (255, 200, 0), thickness)

        if len(self.target_track) >= 2:
            x1, y1, _ = self.target_track[-2]
            x2, y2, _ = self.target_track[-1]

            cv2.arrowedLine(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 255),
                3,
                tipLength=0.25
            )

            dt = max(self.target_track[-1][2] - self.target_track[-2][2], self.cfg.trajectory_min_dt)
            vx = (x2 - x1) / dt
            vy = (y2 - y1) / dt
            speed = math.sqrt(vx * vx + vy * vy)

            cv2.putText(
                frame,
                f"Vitesse: {speed:.1f} px/s",
                (int(x2) + 10, int(y2) + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        for i, (xf, yf) in enumerate(self.predicted_points):
            if 0 <= xf < frame.shape[1] and 0 <= yf < frame.shape[0]:
                radius = max(2, 6 - i // 2)
                cv2.circle(frame, (xf, yf), radius, (255, 0, 255), -1)

        if self.last_target_center is not None and self.predicted_points:
            x0, y0 = self.last_target_center
            x1, y1 = self.predicted_points[0]
            cv2.line(frame, (int(x0), int(y0)), (int(x1), int(y1)), (255, 0, 255), 2)

    # ---------- SCREENSHOTS (PRO FEATURE) ----------
    def _timestamp(self):
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def take_screenshot_auto(self):
        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)
        ts = self._timestamp()

        # Capture the LIVE page UI (radar + camera cards)
        pix = self.page_live.grab()
        ui_path = SCREENSHOT_DIR / f"ui_{ts}.png"
        pix.save(str(ui_path), "PNG")

        # Also save camera frame if available (raw)
        cam_path = None
        if self.last_frame_bgr is not None:
            cam_path = SCREENSHOT_DIR / f"camera_{ts}.png"
            cv2.imwrite(str(cam_path), self.last_frame_bgr)

        # Save config snapshot with the screenshot
        cfg_path = SCREENSHOT_DIR / f"config_{ts}.json"
        cfg_path.write_text(json.dumps(asdict(self.cfg), indent=2), encoding="utf-8")

        if cam_path:
            self.statusBar().showMessage(f"Screenshot saved: {ui_path.name} + {cam_path.name}")
        else:
            self.statusBar().showMessage(f"Screenshot saved: {ui_path.name}")

    def take_screenshot_as(self):
        ts = self._timestamp()
        default = str(SCREENSHOT_DIR / f"ui_{ts}.png")
        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "Save Screenshot", default, "PNG (*.png)")
        if not path:
            return
        pix = self.page_live.grab()
        ok = pix.save(path, "PNG")
        if ok:
            self.statusBar().showMessage(f"Screenshot saved: {Path(path).name}")
        else:
            QMessageBox.critical(self, "Screenshot", "Failed to save screenshot.")

    # ---------- TICKS ----------
    def camera_tick(self):
        if self.cam is None:
            return

        try:
            if self.latest_frame is None:
                return
            frame = self.latest_frame
            self.latest_frame = None

            now = time.time()
            detected_obj = False
            elapsed = 0.0

            if self.cfg.opencv_detect and self.chk_detect.isChecked():
                self.detector_frame_count += 1
                if self.detector_frame_count % max(1, int(self.cfg.detect_every_n_frames)) == 0:
                    # Live page uses lightweight face detection only to keep UI responsive.
                    self.last_det_boxes = []
                    if not self.face_cascade.empty():
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4)
                        detected_obj = (len(faces) > 0)

                        for (x, y, w, h) in faces:
                            x1, y1, x2, y2 = x, y, x + w, y + h
                            self.last_det_boxes.append((x1, y1, x2, y2, "face", True))

                        self.last_det_ts = now

                        if detected_obj:
                            if self.target_start is None:
                                self.target_start = now
                            elapsed = now - self.target_start
                        else:
                            self.target_start = None
                            elapsed = 0.0

            # Keep recent boxes briefly to avoid flicker.
            if self.last_det_boxes and (now - self.last_det_ts) <= self.det_box_ttl:
                for (x1, y1, x2, y2, label, is_target) in self.last_det_boxes:
                    color = (34, 197, 94) if is_target else (150, 150, 150)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # Predictive trajectory
            if self.cfg.trajectory_enabled:
                target_center = self._get_target_center_from_boxes()
                self._update_trajectory(target_center, now)
                self._draw_trajectory_overlay(frame)

            dist_text = f"Dist: {self.last_distance:.1f} cm" if self.last_distance is not None else "Dist: N/A"
            cv2.putText(frame, dist_text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (226, 232, 240), 2)

            status = f"DETECTED: {self.target_object}" if detected_obj else f"Watching for: {self.target_object}"
            color = (0, 255, 0) if detected_obj else (0, 0, 255)
            cv2.putText(frame, status, (16, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # Send buzzer signal via serial (same logic as webcam_detect.py)
            if detected_obj and (now - self.last_buzz_time) > self.buzzer_cooldown:
                if self.ser is not None and getattr(self.ser, "is_open", True):
                    try:
                        self.ser.write(b"1")
                    except Exception:
                        pass
                self.last_buzz_time = now

            if detected_obj and elapsed >= self.cfg.hold_seconds:
                cv2.putText(frame, "CIBLE DETECTEE", (16, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (239, 68, 68), 3)
            else:
                cv2.putText(
                    frame,
                    f"Hold: {elapsed:.1f}s/{self.cfg.hold_seconds:.1f}s",
                    (16, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (226, 232, 240),
                    2
                )

            # store the last frame for saving
            self.last_frame_bgr = frame.copy()
            self.cam_label.setPixmap(frame_to_pixmap(frame))

        except Exception as e:
            self.statusBar().showMessage(f"Camera pipeline error: {e}")
            self.close_camera()

    def serial_tick(self):
        if self.ser is None:
            return

        try:
            if getattr(self.ser, "in_waiting", 0) <= 0:
                return
            raw = self.ser.read(self.ser.in_waiting).decode(errors="replace")
        except Exception:
            return

        self.serial_buffer += raw
        if len(self.serial_buffer) > 4096:
            self.serial_buffer = self.serial_buffer[-4096:]

        got_angle = False
        got_distance = False

        def handle_packet(packet_text: str):
            nonlocal got_angle, got_distance
            packet_text = packet_text.strip()
            if not packet_text:
                return

            # Distance-only stream (e.g. "23.5") from simple Arduino sketches.
            if re.fullmatch(r"-?\d+(?:\.\d+)?", packet_text):
                a, d = None, float(packet_text)
            else:
                a, d = self.parser.feed(packet_text)

            if a is not None:
                self.last_angle_raw = a
                self.last_real_angle_ts = time.time()
                got_angle = True
            if d is not None:
                if d > 0:
                    self.dist_history.append(d)
                valid = [x for x in self.dist_history if x is not None and x > 0]
                smooth = statistics.median(valid) if valid else None
                if smooth is not None:
                    self.last_distance = smooth
                    got_distance = True

        # 1) Newline protocol (safe with decimal distances like 23.5).
        self.serial_buffer = self.serial_buffer.replace("\r", "\n")
        while "\n" in self.serial_buffer:
            line, self.serial_buffer = self.serial_buffer.split("\n", 1)
            handle_packet(line)

        # 2) Processing-style protocol: "angle,distance." without newline.
        pkt_re = re.compile(r"(\d{1,3}\s*,\s*-?\d+(?:\.\d+)?)\.")
        while True:
            m = pkt_re.search(self.serial_buffer)
            if not m:
                break
            handle_packet(m.group(1))
            self.serial_buffer = self.serial_buffer[m.end():]

        # If serial stream has no angle, animate a 0..180..0 sweep like Processing.
        now = time.time()
        if (not got_angle) and ((now - self.last_real_angle_ts) > 0.35):
            dt = max(0.0, now - self.last_synth_update_ts)
            self.last_synth_update_ts = now
            sweep_speed_deg_per_sec = 25.0
            self.synthetic_angle_raw += self.synthetic_dir * sweep_speed_deg_per_sec * dt

            if self.synthetic_angle_raw >= 180.0:
                self.synthetic_angle_raw = 180.0
                self.synthetic_dir = -1.0
            elif self.synthetic_angle_raw <= 0.0:
                self.synthetic_angle_raw = 0.0
                self.synthetic_dir = 1.0

            self.last_angle_raw = self.synthetic_angle_raw

        # Even without new distance values, keep redrawing to show sweep motion.
        if not got_distance and self.last_distance is None:
            self.last_distance = None

        self.radar_widget.update_config(self.cfg)
        self.radar_widget.update_data(self.last_angle_raw, self.last_distance)

        self.cfg.auto_camera_from_radar = self.chk_auto_cam.isChecked()
        self.cfg.opencv_detect = self.chk_detect.isChecked()

        if not self.cfg.auto_camera_from_radar:
            return

        # Do not let radar automation steal the camera when another mode is active.
        if (
            self.image3d_running
            or getattr(self, "object_detection_running", False)
            or getattr(self, "measure_running", False)
            or getattr(self, "face_tracking_running", False)
        ):
            return

        now = time.time()
        detected_now = (self.last_distance is not None) and (self.last_distance <= self.cfg.on_threshold_cm)
        lost_now = (self.last_distance is None) or (self.last_distance >= self.cfg.off_threshold_cm)

        if self.state == "idle":
            if detected_now:
                if self.detect_start is None:
                    self.detect_start = now
                elif (now - self.detect_start) >= self.cfg.trigger_confirm_sec:
                    self.state = "open"
                    self.detect_start = None
                    self.loss_start = None
                    self.statusBar().showMessage("Radar confirmed → opening camera")
                    self.open_camera()
            else:
                self.detect_start = None

        elif self.state == "open":
            if lost_now:
                if self.loss_start is None:
                    self.loss_start = now
                elif (now - self.loss_start) >= self.cfg.cam_close_after_radar_loss:
                    self.statusBar().showMessage("Radar lost → closing camera")
                    self.close_camera()
                    self.state = "idle"
                    self.detect_start = None
                    self.loss_start = None
            else:
                self.loss_start = None

    # ---------- SETTINGS ----------
    def _apply_cfg_to_ui(self):
        self.chk_auto_cam.setChecked(bool(self.cfg.auto_camera_from_radar))
        self.chk_detect.setChecked(bool(self.cfg.opencv_detect))

        self.s_on.setValue(float(self.cfg.on_threshold_cm))
        self.s_off.setValue(float(self.cfg.off_threshold_cm))
        self.s_trigger.setValue(float(self.cfg.trigger_confirm_sec))
        self.s_close.setValue(float(self.cfg.cam_close_after_radar_loss))

        self.s_servo_min.setValue(float(self.cfg.servo_min_angle))
        self.s_servo_max.setValue(float(self.cfg.servo_max_angle))
        self.s_offset.setValue(float(self.cfg.radar_offset_deg))
        self.s_range.setValue(float(self.cfg.radar_max_range_cm))
        self.chk_flip.setChecked(bool(self.cfg.radar_flip))

        self.s_hold.setValue(float(self.cfg.hold_seconds))
        self.s_every.setValue(int(self.cfg.detect_every_n_frames))
        self.s_cam_index.setValue(int(self.cfg.cam_index))
        self.s_image3d_cols.setValue(int(self.cfg.image3d_chessboard_cols))
        self.s_image3d_rows.setValue(int(self.cfg.image3d_chessboard_rows))
        self.s_image3d_square.setValue(float(self.cfg.image3d_square_size_mm))
        self.s_image3d_samples.setValue(int(self.cfg.image3d_min_calibration_frames))
        self.s_image3d_blur.setValue(int(self.cfg.image3d_blur_kernel_size))
        self.s_image3d_area.setValue(int(self.cfg.image3d_min_contour_area))
        self.chk_image3d_debug.setChecked(bool(self.cfg.image3d_debug))
        self.image3d_chk_debug.setChecked(bool(self.cfg.image3d_debug))

        # Target object selection
        if hasattr(self, "cmb_target"):
            self.cmb_target.setCurrentText(str(self.cfg.target_object))

        self._set_connection_pill(self.ser is not None)

    def apply_settings(self):
        self.cfg.on_threshold_cm = float(self.s_on.value())
        self.cfg.off_threshold_cm = float(self.s_off.value())
        self.cfg.trigger_confirm_sec = float(self.s_trigger.value())
        self.cfg.cam_close_after_radar_loss = float(self.s_close.value())

        self.cfg.servo_min_angle = float(self.s_servo_min.value())
        self.cfg.servo_max_angle = float(self.s_servo_max.value())
        self.cfg.radar_offset_deg = float(self.s_offset.value())
        self.cfg.radar_max_range_cm = float(self.s_range.value())
        self.cfg.radar_flip = bool(self.chk_flip.isChecked())

        self.cfg.hold_seconds = float(self.s_hold.value())
        self.cfg.detect_every_n_frames = int(self.s_every.value())
        self.cfg.cam_index = int(self.s_cam_index.value())
        self.cfg.image3d_chessboard_cols = int(self.s_image3d_cols.value())
        self.cfg.image3d_chessboard_rows = int(self.s_image3d_rows.value())
        self.cfg.image3d_square_size_mm = float(self.s_image3d_square.value())
        self.cfg.image3d_min_calibration_frames = int(self.s_image3d_samples.value())
        self.cfg.image3d_blur_kernel_size = int(self.s_image3d_blur.value())
        self.cfg.image3d_min_contour_area = int(self.s_image3d_area.value())
        self.cfg.image3d_debug = bool(self.chk_image3d_debug.isChecked())

        if hasattr(self, "cmb_target"):
            self.cfg.target_object = str(self.cmb_target.currentText()).strip()
            self.target_object = self.cfg.target_object

        self.cfg.auto_camera_from_radar = bool(self.chk_auto_cam.isChecked())
        self.cfg.opencv_detect = bool(self.chk_detect.isChecked())
        self.image3d_chk_debug.setChecked(bool(self.cfg.image3d_debug))

        self.radar_widget.update_config(self.cfg)
        self._refresh_image3d_backend()
        self._update_image3d_status_labels()
        self.statusBar().showMessage("Settings applied")

    def save_config(self):
        self.apply_settings()
        try:
            self.cfg.save(CONFIG_PATH)
            self.statusBar().showMessage(f"Config saved: {CONFIG_PATH.name}")
        except Exception as e:
            QMessageBox.critical(self, "Save config", str(e))

    def open_config_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open config", str(APP_DIR), "JSON (*.json)")
        if not path:
            return
        try:
            self.cfg = AppConfig.load(Path(path))
            self._apply_cfg_to_ui()
            self.radar_widget.update_config(self.cfg)
            self.statusBar().showMessage("Config loaded")
        except Exception as e:
            QMessageBox.critical(self, "Open config", str(e))

    # ---------- CLEANUP ----------
    def closeEvent(self, event):
        self.close_camera()
        self.stop_image3d_camera()
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    apply_pro_theme(app)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()