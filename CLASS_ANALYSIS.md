# Comprehensive Class Analysis - Projet Suivi Cible VersionFinal

**Analysis Date:** May 6, 2026  
**Scope:** All Python files in the project

---

## SUMMARY
- **Total Classes Found:** 12
- **USED Classes:** 9
- **UNUSED Classes:** 3
- **Inner Classes:** 1

---

## DETAILED CLASS INVENTORY

### FILE: python/core_radar.py

#### 1. **RadarDataManager**
- **Location:** [core_radar.py](core_radar.py#L11), Line 11
- **Status:** ❌ **UNUSED**
- **Methods:**
  - `__init__(self, config: AppConfig)` - Constructor
  - `update_config(self, config: AppConfig) -> None` - Update configuration
  - `map_angle(self, angle_deg: float) -> float` - Map raw servo angle with calibration
  - `update_data(self, angle_raw: float = None, dist_cm: float = None) -> None` - Update radar data
  - `get_stale_age(self) -> float` - Get time before point becomes stale
  - `get_sweep_data(self) -> tuple` - Get current sweep and point data
- **Additional Details:** Has 6 methods beyond __init__. Maintains deques for points and sweep angles. Never imported or used anywhere in the codebase.

#### 2. **RadarState**
- **Location:** [core_radar.py](core_radar.py#L90), Line 90
- **Status:** ❌ **UNUSED**
- **Methods:**
  - `__init__(self, config: AppConfig)` - Constructor
  - `reset(self) -> None` - Reset state
  - `update(self, distance: float = None) -> str` - Update state machine, returns action
- **Additional Details:** Has 3 methods beyond __init__. Implements state machine logic for camera automation. Never imported or used anywhere.

---

### FILE: python/core_yolo.py

#### 3. **YOLODetector**
- **Location:** [core_yolo.py](core_yolo.py#L14), Line 14
- **Status:** ❌ **UNUSED**
- **Methods:**
  - `__init__(self)` - Constructor
  - `is_available()` - Static method to check if ultralytics module installed
  - `load_model(self, model_name: str = "yolov8m.pt") -> bool` - Load YOLO model
  - `open_camera(self, cam_index: int = 0) -> bool` - Open camera for detection
  - `close_camera(self) -> None` - Close camera
  - `is_ready(self) -> bool` - Check if detector is ready
  - `detect_frame(self, frame=None) -> dict` - Run inference on frame
  - `cleanup(self) -> None` - Clean up resources
- **Additional Details:** Has 8 methods beyond __init__. Complete YOLO detection wrapper. Never imported or instantiated anywhere in the codebase.

---

### FILE: python/image_3d.py

#### 4. **Image3DConfig**
- **Location:** [image_3d.py](image_3d.py#L12), Line 12
- **Status:** ✅ **USED**
- **Type:** @dataclass (from dataclasses)
- **Methods:** 
  - None (standard dataclass methods only)
- **Additional Details:** Has 6 fields (chessboard_cols, chessboard_rows, square_size_mm, min_calibration_frames, blur_kernel_size, min_contour_area). Created in gui_app.py _build_image3d_config() at line 746. Imported in gui_app.py at line 30.

#### 5. **Image3DState**
- **Location:** [image_3d.py](image_3d.py#L22), Line 22
- **Status:** ✅ **USED**
- **Type:** @dataclass (from dataclasses)
- **Methods:**
  - None (standard dataclass methods only)
- **Additional Details:** Has 7 fields (frame_bgr, width_mm, length_mm, estimated_height_mm, rms_error, calibration_samples, x_board_mm, y_board_mm, message). Instantiated in SingleCameraChessboardMeasurer.process_frame() at line 139.

#### 6. **SingleCameraChessboardMeasurer**
- **Location:** [image_3d.py](image_3d.py#L34), Line 34
- **Status:** ✅ **USED**
- **Methods:**
  - `__init__(self, config: Image3DConfig, calibration_file: Path | str)` - Constructor
  - `capture_calibration_sample(self, frame_bgr: np.ndarray)` - Capture calibration sample
  - `calibrate(self)` - Calibrate camera
  - `reset_calibration_samples(self)` - Reset captured samples
  - `delete_saved_calibration(self)` - Delete saved calibration
  - `process_frame(self, frame_bgr: np.ndarray, debug: bool = False)` - Process frame
  - `_build_chessboard_object_points(self)` - Build chessboard points
  - `_find_chessboard(self, gray: np.ndarray)` - Find chessboard
  - `_refine_corners(self, gray: np.ndarray, corners: np.ndarray)` - Refine corner detection
  - `_compute_homography_from_corners(self, corners: np.ndarray)` - Compute homography
  - `_board_polygon_from_homography(self, H: np.ndarray)` - Get board polygon
  - `_detect_object_using_board_reference(self, frame_bgr, H, board_polygon)` - Detect object
  - `_estimate_height_mm(self, rect)` - Estimate height
  - `_save_calibration(self)` - Save calibration
  - `_load_saved_calibration(self)` - Load saved calibration
- **Additional Details:** Has 15 methods beyond __init__ (3 public, 12 private). Imported in gui_app.py at line 30. Instantiated in MainWindow.__init__() at line 552 and _refresh_image3d_backend() at line 757.

---

### FILE: python/gui_app.py

#### 7. **AppConfig**
- **Location:** [gui_app.py](gui_app.py#L39), Line 39
- **Status:** ✅ **USED**
- **Type:** @dataclass (from dataclasses)
- **Methods:**
  - `save(self, path: Path = CONFIG_PATH)` - Save configuration to JSON
  - `load(path: Path = CONFIG_PATH)` - Static method to load configuration
- **Additional Details:** Has 25 configuration fields (baud, serial_timeout_sec, thresholds, angles, etc.). Used throughout MainWindow. Instantiated in MainWindow.__init__() at line 490 via AppConfig.load(). Saved/loaded in multiple places.

#### 8. **SerialParser**
- **Location:** [gui_app.py](gui_app.py#L235), Line 235
- **Status:** ✅ **USED**
- **Methods:**
  - `__init__(self)` - Constructor
  - `feed(self, line: str)` - Parse serial line, return (angle, distance)
- **Additional Details:** Has 1 method beyond __init__. Simple parser for serial data. Instantiated in MainWindow.__init__() at line 497 and connect_serial() at line 1932.

#### 9. **RadarWidget**
- **Location:** [gui_app.py](gui_app.py#L273), Line 273
- **Status:** ✅ **USED**
- **Type:** Inherits from QWidget
- **Methods:**
  - `__init__(self, cfg: AppConfig)` - Constructor
  - `update_config(self, cfg: AppConfig)` - Update configuration
  - `map_angle(self, angle_deg: float) -> float` - Map raw servo angle
  - `update_data(self, angle_raw, dist_cm)` - Update radar data
  - `paintEvent(self, event)` - Paint radar visualization
- **Additional Details:** Has 5 methods including __init__. Custom PyQt6 widget for radar visualization. Instantiated in _build_live_page() at line 1704.

#### 10. **CameraCaptureThread**
- **Location:** [gui_app.py](gui_app.py#L411), Line 411
- **Status:** ✅ **USED**
- **Type:** Inherits from QThread
- **Methods:**
  - `__init__(self, cam_index: int)` - Constructor
  - `stop(self)` - Stop the thread
  - `_open_camera(self)` - Open camera with multiple backends
  - `run(self)` - Main thread loop
- **Additional Details:** Has 4 methods including __init__. Signals: frame_ready, camera_error. Instantiated in open_camera() at line 2080.

#### 11. **MainWindow**
- **Location:** [gui_app.py](gui_app.py#L482), Line 482
- **Status:** ✅ **USED**
- **Type:** Inherits from QMainWindow
- **Methods:** (Partial list - large class with 60+ methods)
  - `__init__(self)` - Constructor
  - `goto_*()` - Navigation methods (goto_home, goto_live, goto_image_3d, etc.)
  - `_build_*()` - UI building methods
  - `*_tick()` - Timed update methods (camera_tick, serial_tick)
  - `start_*_camera()`, `stop_*_camera()` - Camera control methods
  - `toggle_*()` - Mode toggle methods
  - `apply_settings()` - Apply settings
  - `save_config()`, `open_config_dialog()` - Config management
  - And many more...
- **Additional Details:** Has 60+ methods. Main application window. Instantiated in main() function at line 2585.

---

### FILE: python/main.py

#### 12. **FakeSerial** (Inner Class)
- **Location:** [main.py](main.py#L147), Line 147
- **Status:** ✅ **USED**
- **Type:** Inner class defined inside main() function
- **Methods:**
  - `__init__(self, timeline)` - Constructor
  - `readline(self)` - Simulate serial readline
- **Additional Details:** Has 2 methods including __init__. Mock serial connection for testing. Instantiated in main() at line 161 when simulate=True.

---

## UNUSED CLASSES SUMMARY

Three classes are completely unused and can be removed:

### 1. RadarDataManager (core_radar.py:11)
- **Recommendation:** DELETE - Never imported or used. Functionality is partially replicated in RadarWidget.
- **Reason:** Duplicate functionality; RadarWidget handles the same operations inline.

### 2. RadarState (core_radar.py:90)
- **Recommendation:** DELETE - Never imported or used. State machine logic exists in MainWindow.serial_tick().
- **Reason:** Duplicate functionality; MainWindow implements its own state machine for camera automation.

### 3. YOLODetector (core_yolo.py:14)
- **Recommendation:** DELETE - Never imported or used anywhere.
- **Reason:** Although it's a well-designed class, it's never instantiated. The YOLO model is loaded directly in MainWindow with:
  ```python
  if YOLO is not None:
      try:
          model_path = Path(__file__).resolve().parents[2] / "yolov8n.pt"
          self.yolo = YOLO(str(model_path))
  ```
  The YOLODetector wrapper is unnecessary overhead.

---

## NOTES ON FILE ANALYSIS

### Files with No Classes:
- **python/camera_test.py** - Test script only, no classes
- **python/detect_ports.py** - Utility script only, no classes
- **python/face_tracking.py** - Utility script with functions only, no classes
- **python/test_arduino.py** - Test script only, no classes
- **python/gui_app_legacy.py** - Empty file

### Subdirectories:
- **python/services/** - Empty (only __pycache__)
- **python/ui/** - Empty (only __pycache__)
- **python/widgets/** - Empty (only __pycache__)

---

## DEPENDENCIES & IMPORTS

### Core Dependencies:
- `AppConfig` is a foundational class used throughout the entire application
- `MainWindow` depends on AppConfig, RadarWidget, CameraCaptureThread, Image3DConfig, SingleCameraChessboardMeasurer
- `Image3DConfig` and `Image3DState` are tightly coupled with `SingleCameraChessboardMeasurer`

### Import Pattern:
- gui_app.py imports: `Image3DConfig`, `SingleCameraChessboardMeasurer` from image_3d.py
- core_radar.py defines but never imports: `RadarDataManager`, `RadarState`
- core_yolo.py defines but never imports: `YOLODetector`

---

## RECOMMENDATIONS

1. **Remove unused classes:**
   - Delete `RadarDataManager` from core_radar.py
   - Delete `RadarState` from core_radar.py
   - Delete entire `YOLODetector` class from core_yolo.py OR repurpose for future use

2. **Consolidate radar logic:**
   - If RadarDataManager is to be kept, refactor MainWindow to use it instead of inline logic

3. **Consider refactoring:**
   - Extract YOLODetector as a proper wrapper for better code organization
   - Move FakeSerial to a test utilities module instead of inline in main()

---

## END OF ANALYSIS

**Total Methods Across All USED Classes:** 95+ methods  
**Total Methods Across All UNUSED Classes:** 14 methods  
**Code Duplication Risk:** HIGH (RadarDataManager/RadarState duplicate MainWindow logic)
