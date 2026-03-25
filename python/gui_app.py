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
except ImportError:
    YOLO = None
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont, QAction
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QFrame, QMessageBox, QStackedWidget,
    QCheckBox, QSlider, QSpinBox, QDoubleSpinBox, QFileDialog,
    QLineEdit, QCompleter,
    QGridLayout
)

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

    radar_max_range_cm: float = 200.0
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

    def feed(self, packet: str):
        if not packet:
            return None, None
        s = packet.strip()
        low = s.lower()

        # Arduino + Processing format: "angle,distance."
        m = re.match(r"^\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*$", s)
        if m:
            self.pending_angle = None
            return float(m.group(1)), float(m.group(2))

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
        self.target_sweep_angle = 90.0
        self.current_sweep_angle = 90.0
        self._last_anim_ts = time.time()

        self.sweep_angles = deque(maxlen=self.cfg.sweep_trail)
        self.points = deque(maxlen=600)  # (angle, dist, timestamp)

        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._animate_sweep)
        self.anim_timer.start(16)

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
            self.target_sweep_angle = self.last_angle

            if not self.sweep_angles:
                self.current_sweep_angle = self.last_angle
                self.sweep_angles.appendleft(self.current_sweep_angle)

        if dist_cm is not None:
            self.last_distance = float(dist_cm)
            if 0.0 < self.last_distance <= self.cfg.radar_max_range_cm:
                self.points.append((self.last_angle, self.last_distance, now))

        self.update()

    def _animate_sweep(self):
        now = time.time()
        dt = min(0.06, max(0.001, now - self._last_anim_ts))
        self._last_anim_ts = now

        delta = self.target_sweep_angle - self.current_sweep_angle
        max_speed_deg_s = 420.0
        max_step = max_speed_deg_s * dt

        if abs(delta) <= max_step:
            self.current_sweep_angle = self.target_sweep_angle
        else:
            self.current_sweep_angle += max_step if delta > 0 else -max_step

        self.sweep_angles.appendleft(self.current_sweep_angle)
        self.update()

    def paintEvent(self, event):
        now = time.time()
        while self.points and (now - self.points[0][2]) > self.cfg.point_ttl_sec:
            self.points.popleft()

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(11, 15, 23))

        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.93
        radius = min(w * 0.48, h * 0.85)

        grid = QColor(34, 197, 94, 90)
        sweep = QColor(34, 197, 94, 220)

        p.setPen(QPen(grid, 2))
        for frac in [0.25, 0.5, 0.75, 1.0]:
            r = radius * frac
            p.drawArc(int(cx - r), int(cy - r), int(2 * r), int(2 * r), 0 * 16, 180 * 16)

        for ang in [0, 30, 60, 90, 120, 150, 180]:
            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(int(cx), int(cy), int(x), int(y))

        p.setPen(QPen(QColor(226, 232, 240)))
        p.setFont(QFont("Consolas", 10))
        dist_txt = f"{self.last_distance:.0f} cm" if self.last_distance is not None else "N/A"
        p.drawText(12, 22, f"Angle: {self.last_angle:.0f}°   Dist: {dist_txt}")

        for i, ang in enumerate(self.sweep_angles):
            alpha = max(20, 230 - i * 12)
            p.setPen(QPen(QColor(sweep.red(), sweep.green(), sweep.blue(), alpha), 4 if i == 0 else 2))
            rad = math.radians(180 - ang)
            x = cx + radius * math.cos(rad)
            y = cy - radius * math.sin(rad)
            p.drawLine(QPointF(cx, cy), QPointF(x, y))

        for (ang, dist, ts) in self.points:
            age = now - ts
            fade = max(40, int(255 * (1.0 - age / self.cfg.point_ttl_sec)))
            p.setPen(QPen(QColor(239, 68, 68, fade), 6))

            r = (dist / self.cfg.radar_max_range_cm) * radius
            rad = math.radians(180 - ang)
            x = cx + r * math.cos(rad)
            y = cy - r * math.sin(rad)
            p.drawPoint(int(x), int(y))


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
        self.serial_buffer = ""
        self.parser = SerialParser()
        self.last_angle_raw = None
        self.last_distance = None
        self.dist_history = deque(maxlen=5)

        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

        self.cam = None
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

        # Keep latest inference boxes for a short time to avoid flicker when running detection every N frames
        self.last_det_boxes = []  # list of (x1,y1,x2,y2,label,is_target)
        self.last_det_ts = 0.0
        self.det_box_ttl = 0.6  # seconds

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
        self.page_object_detection = self._build_object_detection_page()
        self.page_settings = self._build_settings_page()
        self.page_about = self._build_about_page()

        self.pages.addWidget(self.page_home)           # 0
        self.pages.addWidget(self.page_live)           # 1
        self.pages.addWidget(self.page_face_tracking)  # 2
        self.pages.addWidget(self.page_measure)        # 3
        self.pages.addWidget(self.page_object_detection)  # 4
        self.pages.addWidget(self.page_settings)       # 5
        self.pages.addWidget(self.page_about)          # 6

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
        view_menu.addAction(QAction("Radar", self, triggered=self.goto_live))
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
        self.btn_live = self._nav_button("  📡  Radar", self.goto_live)
        self.btn_face_tracking = self._nav_button("  🎯  Face Tracking", self.goto_face_tracking)
        self.btn_measure = self._nav_button("  📏  Mesure d'objet", self.goto_measure)
        self.btn_object_detection = self._nav_button("  🔎  Object Detection", self.goto_object_detection)
        self.btn_settings = self._nav_button("  ⚙️  Settings", self.goto_settings)
        self.btn_about = self._nav_button("  ℹ️  About", self.goto_about)

        lay.addWidget(self.btn_home)
        lay.addWidget(self.btn_live)
        lay.addWidget(self.btn_face_tracking)
        lay.addWidget(self.btn_measure)
        lay.addWidget(self.btn_object_detection)
        lay.addWidget(self.btn_settings)
        lay.addWidget(self.btn_about)
        lay.addStretch()

        foot = QLabel("v1.1 • ProjetProg")
        foot.setObjectName("SidebarSmall")

        lay.addWidget(foot)
        return sidebar

    def _set_nav_checked(self, which: str):
        for b in [self.btn_home, self.btn_live, self.btn_face_tracking, self.btn_measure, self.btn_object_detection, self.btn_settings, self.btn_about]:
            b.setChecked(False)
        getattr(self, which).setChecked(True)

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

    def goto_measure(self):
        self.pages.setCurrentIndex(3)
        self.lbl_title.setText("Mesure d'objet")
        self.lbl_sub.setText("Mesure avancée : calibration, contours, surface")
        self._set_nav_checked("btn_measure")

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
        self.pages.setCurrentIndex(4)
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
        self.apply_object_detection_target()
        if self.yolo is None:
            self.object_detection_label.setText("YOLO model unavailable")
            self.statusBar().showMessage("Unable to start object detection: YOLO model unavailable")
            return

        self.object_detection_cap = cv2.VideoCapture(self.cfg.cam_index)
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
        self.measure_cap = cv2.VideoCapture(self.cfg.cam_index)
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
            "App pro: menu, pages, settings, capture d’écran et config.\n"
            "• Live: radar + caméra + OpenCV\n"
            "• Settings: seuils, calibration, performance\n"
            "• Screenshot: preuve visuelle pour la démo (dossier screenshots/)."
        )
        msg.setStyleSheet("color:#94a3b8;")
        h.addWidget(msg)

        btn_row = QHBoxLayout()
        b1 = QPushButton("Go to Radar")
        b1.clicked.connect(self.goto_live)
        b2 = QPushButton("Open Settings")
        b2.setObjectName("SecondaryBtn")
        b2.clicked.connect(self.goto_settings)
        btn_row.addWidget(b1)
        btn_row.addWidget(b2)
        btn_row.addStretch()
        h.addLayout(btn_row)

        lay.addWidget(hero)
        lay.addStretch()
        return page

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
        self.s_baud = QComboBox()
        for b in (9600, 19200, 38400, 57600, 115200):
            self.s_baud.addItem(str(b), b)
        self.s_baud.setToolTip("Doit correspondre au Serial.begin(...) dans le sketch Arduino")

        gl.addWidget(QLabel("ON threshold"), 0, 0); gl.addWidget(self.s_on, 0, 1)
        gl.addWidget(QLabel("OFF threshold"), 1, 0); gl.addWidget(self.s_off, 1, 1)
        gl.addWidget(QLabel("Confirm time"), 2, 0); gl.addWidget(self.s_trigger, 2, 1)
        gl.addWidget(QLabel("Close after loss"), 3, 0); gl.addWidget(self.s_close, 3, 1)
        gl.addWidget(QLabel("Serial baud"), 4, 0); gl.addWidget(self.s_baud, 4, 1)

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
        self.lbl_title.setText("Radar")
        self.lbl_sub.setText("Radar en temps réel + caméra + OpenCV")
        self._set_nav_checked("btn_live")

    def goto_settings(self):
        self.pages.setCurrentIndex(4)
        self.lbl_title.setText("Settings")
        self.lbl_sub.setText("Seuils • calibration • performance • config")
        self._set_nav_checked("btn_settings")

    def goto_about(self):
        self.pages.setCurrentIndex(5)
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
            self.serial_buffer = ""
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
        self.serial_buffer = ""
        self.dist_history.clear()
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None

    # ---------- CAMERA ----------
    def toggle_camera(self):
        if self.cam is None:
            self.open_camera()
        else:
            self.close_camera()

    def open_camera(self):
        cam = cv2.VideoCapture(int(self.cfg.cam_index))
        if not cam.isOpened():
            QMessageBox.critical(self, "Camera", f"Camera not accessible (index {self.cfg.cam_index}).")
            return
        self.cam = cam
        self.btn_camera.setText("Stop camera")
        self.target_start = None
        self.last_det_boxes = []
        self.last_det_ts = 0.0


    def close_camera(self):
        try:
            if self.cam:
                self.cam.release()
        except Exception:
            pass
        self.cam = None
        self.btn_camera.setText("Start camera")
        self.cam_label.setText("Camera stopped")
        self.target_start = None
        self.last_frame_bgr = None

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

        ret, frame = self.cam.read()
        if not ret:
            return

        now = time.time()
        detected_obj = False
        elapsed = 0.0

        if self.cfg.opencv_detect and self.chk_detect.isChecked():
            self.detector_frame_count += 1
            if self.detector_frame_count % max(1, int(self.cfg.detect_every_n_frames)) == 0:
                # Use YOLOv8 to detect objects (e.g. "cell phone")
                if self.yolo is not None:
                    results = self.yolo(frame, verbose=False)
                    detections = results[0]

                    # Store detected boxes so we can keep them on-screen for a short time
                    self.last_det_boxes = []
                    for box in detections.boxes:
                        class_id = int(box.cls[0])
                        label = self.yolo.names.get(class_id, str(class_id))
                        is_target = (label == self.target_object)
                        if is_target:
                            detected_obj = True
                        try:
                            xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        except Exception:
                            xyxy = box.xyxy[0].astype(int)
                        x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

                        self.last_det_boxes.append((x1, y1, x2, y2, label, is_target))

                    self.last_det_ts = now

                    if detected_obj:
                        if self.target_start is None:
                            self.target_start = now
                        elapsed = now - self.target_start
                    else:
                        self.target_start = None
                        elapsed = 0.0

        # If we haven't run detection this frame, still draw the last boxes briefly to avoid flicker.
        if self.last_det_boxes and (now - self.last_det_ts) <= self.det_box_ttl:
            for (x1, y1, x2, y2, label, is_target) in self.last_det_boxes:
                color = (34, 197, 94) if is_target else (150, 150, 150)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Fallback to Haar cascade if YOLO is not available
        elif not self.face_cascade.empty():
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=4)
            detected_obj = (len(faces) > 0)

            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (34, 197, 94), 2)

            if detected_obj:
                if self.target_start is None:
                    self.target_start = now
                elapsed = now - self.target_start
            else:
                self.target_start = None
                elapsed = 0.0


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
            cv2.putText(frame, f"Hold: {elapsed:.1f}s/{self.cfg.hold_seconds:.1f}s", (16, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (226, 232, 240), 2)

        # store the last frame for saving
        self.last_frame_bgr = frame.copy()

        self.cam_label.setPixmap(frame_to_pixmap(frame))

    def serial_tick(self):
        if self.ser is None:
            return

        try:
            waiting = int(getattr(self.ser, "in_waiting", 0))
            if waiting <= 0:
                return
            chunk = self.ser.read(waiting).decode(errors="replace")
        except Exception:
            return

        self.serial_buffer += chunk
        if len(self.serial_buffer) > 8192:
            self.serial_buffer = self.serial_buffer[-4096:]

        had_update = False

        def process_message(msg: str):
            nonlocal had_update
            a, d = self.parser.feed(msg)

            if a is not None:
                self.last_angle_raw = a

            if d is not None:
                if d > 0:
                    self.dist_history.append(d)
                    valid = [x for x in self.dist_history if x is not None and x > 0]
                    smooth = statistics.median(valid) if valid else None
                    if smooth is not None:
                        self.last_distance = smooth
                else:
                    # HC-SR04 often returns -1 or 0 when no echo is received.
                    self.last_distance = None

            if (a is not None) or (d is not None):
                self.radar_widget.update_data(self.last_angle_raw, self.last_distance)
                had_update = True

        # Support Processing-style stream: "angle,distance." packets.
        if ("," in self.serial_buffer) and ("." in self.serial_buffer):
            packets = self.serial_buffer.split(".")
            self.serial_buffer = packets.pop() if packets else ""
            for packet in packets:
                packet = packet.strip()
                if packet:
                    process_message(packet)

        # Support line-based stream: Serial.println(...)
        elif ("\n" in self.serial_buffer) or ("\r" in self.serial_buffer):
            normalized = self.serial_buffer.replace("\r", "\n")
            lines = normalized.split("\n")
            self.serial_buffer = lines.pop() if lines else ""
            for line in lines:
                line = line.strip()
                if line:
                    process_message(line)

        if not had_update:
            self.radar_widget.update_data(self.last_angle_raw, self.last_distance)

        self.cfg.auto_camera_from_radar = self.chk_auto_cam.isChecked()
        self.cfg.opencv_detect = self.chk_detect.isChecked()

        if not self.cfg.auto_camera_from_radar:
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

        if hasattr(self, "s_baud"):
            idx = self.s_baud.findData(int(self.cfg.baud))
            if idx >= 0:
                self.s_baud.setCurrentIndex(idx)

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

        # Target object selection
        if hasattr(self, "cmb_target"):
            self.cmb_target.setCurrentText(str(self.cfg.target_object))

        self._set_connection_pill(self.ser is not None)

    def apply_settings(self):
        if hasattr(self, "s_baud"):
            self.cfg.baud = int(self.s_baud.currentData())

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

        if hasattr(self, "cmb_target"):
            self.cfg.target_object = str(self.cmb_target.currentText()).strip()
            self.target_object = self.cfg.target_object

        self.cfg.auto_camera_from_radar = bool(self.chk_auto_cam.isChecked())
        self.cfg.opencv_detect = bool(self.chk_detect.isChecked())

        self.radar_widget.update_config(self.cfg)
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