import sys
import serial
import serial.tools.list_ports

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QGroupBox, QCheckBox, QMessageBox
)

BAUD = 115200
SERIAL_TIMEOUT_SEC = 0.2


def list_com_ports():
    ports = list(serial.tools.list_ports.comports())
    return [(p.device, p.description) for p in ports]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Projet Suivi Cible - App")
        self.resize(1100, 650)

        # Serial object
        self.ser = None

        # --- central layout ---
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ===== Top bar (controls) =====
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

        self.btn_camera = QPushButton("Start camera (later)")
        self.btn_camera.setEnabled(False)  # on l’activera quand on branchera OpenCV

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

        # ===== Content area =====
        content = QHBoxLayout()

        self.radar_box = QGroupBox("Radar")
        radar_layout = QVBoxLayout(self.radar_box)
        self.radar_placeholder = QLabel("Radar view (placeholder)")
        self.radar_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.radar_placeholder.setStyleSheet("background:#111; color:#00ff66; padding:10px;")
        radar_layout.addWidget(self.radar_placeholder)

        self.cam_box = QGroupBox("Camera")
        cam_layout = QVBoxLayout(self.cam_box)
        self.cam_placeholder = QLabel("Camera view (placeholder)")
        self.cam_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cam_placeholder.setStyleSheet("background:#111; color:white; padding:10px;")
        cam_layout.addWidget(self.cam_placeholder)

        content.addWidget(self.radar_box, 1)
        content.addWidget(self.cam_box, 1)

        main_layout.addLayout(content)

        # ===== Status bar =====
        self.statusBar().showMessage("Ready")

        # Load ports now
        self.refresh_ports()

        # Apply default view
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
                self,
                "Serial",
                f"Accès refusé à {port}.\n\n"
                "Ferme Arduino Serial Monitor / VS Code Serial Monitor / autres scripts Python,\n"
                "puis réessaie."
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

    def apply_view_mode(self):
        mode = self.view_combo.currentText()
        self.radar_box.setVisible(mode in ["Both", "Radar only"])
        self.cam_box.setVisible(mode in ["Both", "Camera only"])

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()