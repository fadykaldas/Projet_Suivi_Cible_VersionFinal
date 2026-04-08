from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class Image3DConfig:
    """Configuration for chessboard-based single-camera measurement."""

    chessboard_cols: int = 9
    chessboard_rows: int = 6
    square_size_mm: float = 25.0
    min_calibration_frames: int = 12
    blur_kernel_size: int = 5
    canny_low: int = 50
    canny_high: int = 150
    min_contour_area: int = 2000
    max_contour_area_fraction: float = 0.75

    @property
    def chessboard_size(self) -> tuple[int, int]:
        return (int(self.chessboard_cols), int(self.chessboard_rows))


@dataclass
class Image3DState:
    """Runtime state returned to the GUI for display and interaction."""

    frame_bgr: np.ndarray
    board_visible: bool
    object_visible: bool
    message: str
    width_mm: Optional[float] = None
    length_mm: Optional[float] = None
    estimated_height_mm: Optional[float] = None
    rms_error: Optional[float] = None
    calibration_samples: int = 0


class SingleCameraChessboardMeasurer:
    """Backend for calibration and live measurement on a known chessboard plane.

    Width and length are measured on the chessboard plane in millimeters.
    Height is only a best-effort estimate because a single RGB camera does not
    provide reliable depth without extra assumptions.
    """

    def __init__(self, config: Image3DConfig, calibration_path: Path):
        self.config = config
        self.calibration_path = Path(calibration_path)
        self.subpix_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        self.object_points: list[np.ndarray] = []
        self.image_points: list[np.ndarray] = []
        self.frame_size: Optional[tuple[int, int]] = None

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.rms_error: Optional[float] = None

        self._board_object_points = self._build_board_object_points()
        self.load_calibration()

    def _build_board_object_points(self) -> np.ndarray:
        """Return chessboard inner-corner coordinates in millimeters."""
        cols, rows = self.config.chessboard_size
        object_points = np.zeros((rows * cols, 3), dtype=np.float32)
        object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        object_points *= float(self.config.square_size_mm)
        return object_points

    def load_calibration(self) -> bool:
        """Load saved calibration data if available."""
        if not self.calibration_path.exists():
            return False

        try:
            data = np.load(self.calibration_path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            self.rms_error = float(data.get("rms", 0.0))
            return True
        except Exception:
            self.camera_matrix = None
            self.dist_coeffs = None
            self.rms_error = None
            return False

    def reset_calibration_samples(self) -> None:
        """Clear captured calibration views without deleting saved calibration."""
        self.object_points.clear()
        self.image_points.clear()
        self.frame_size = None

    def delete_saved_calibration(self) -> None:
        """Remove saved calibration from disk and memory."""
        if self.calibration_path.exists():
            self.calibration_path.unlink()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rms_error = None
        self.reset_calibration_samples()

    def detect_chessboard(self, frame_bgr: np.ndarray) -> tuple[bool, Optional[np.ndarray]]:
        """Detect and refine chessboard corners on a BGR frame."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, self.config.chessboard_size, flags)
        if not found:
            return False, None

        refined = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=self.subpix_criteria,
        )
        return True, refined

    def capture_calibration_sample(self, frame_bgr: np.ndarray) -> tuple[bool, str]:
        """Capture one calibration sample when the chessboard is visible."""
        found, corners = self.detect_chessboard(frame_bgr)
        if not found or corners is None:
            return False, "Chessboard not found. Show the full board and try again."

        self.object_points.append(self._board_object_points.copy())
        self.image_points.append(corners)
        self.frame_size = (frame_bgr.shape[1], frame_bgr.shape[0])
        count = len(self.image_points)
        return True, f"Calibration sample saved ({count}/{self.config.min_calibration_frames} minimum)."

    def calibrate(self) -> tuple[bool, str]:
        """Run cv2.calibrateCamera using captured chessboard images."""
        if len(self.image_points) < self.config.min_calibration_frames:
            return False, (
                f"Need at least {self.config.min_calibration_frames} calibration views; "
                f"currently have {len(self.image_points)}."
            )
        if self.frame_size is None:
            return False, "Calibration frame size is unknown. Capture again."

        try:
            rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
                self.object_points,
                self.image_points,
                self.frame_size,
                None,
                None,
            )
        except Exception as exc:
            return False, f"Calibration failed: {exc}"

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rms_error = float(rms)
        np.savez(
            self.calibration_path,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            rms=self.rms_error,
        )
        return True, f"Calibration saved. RMS error: {self.rms_error:.4f} px"

    def undistort(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Undistort a frame using the loaded calibration, if available."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return frame_bgr

        h, w = frame_bgr.shape[:2]
        new_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            (w, h),
            1,
            (w, h),
        )
        undistorted = cv2.undistort(frame_bgr, self.camera_matrix, self.dist_coeffs, None, new_matrix)
        x, y, roi_w, roi_h = roi
        if roi_w > 0 and roi_h > 0:
            undistorted = undistorted[y:y + roi_h, x:x + roi_w]
        return undistorted

    def _estimate_pose(self, corners: np.ndarray) -> tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate chessboard pose with solvePnP."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return False, None, None

        success, rvec, tvec = cv2.solvePnP(
            self._board_object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return False, None, None
        return True, rvec, tvec

    def _board_mask(self, shape: tuple[int, int, int], corners: np.ndarray) -> np.ndarray:
        """Create a mask covering the chessboard so it is not detected as the object."""
        mask = np.zeros(shape[:2], dtype=np.uint8)
        pts = corners.reshape(-1, 2).astype(np.int32)
        hull = cv2.convexHull(pts)
        cv2.fillConvexPoly(mask, hull, 255)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (24, 24))
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def _detect_object_contour(
        self,
        frame_bgr: np.ndarray,
        board_mask: Optional[np.ndarray],
        debug: bool,
    ) -> tuple[Optional[np.ndarray], dict[str, np.ndarray]]:
        """Detect the largest valid foreground contour."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        kernel_size = int(self.config.blur_kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1
        blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        threshold = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11,
            2,
        )
        edges = cv2.Canny(blurred, self.config.canny_low, self.config.canny_high)
        combined = cv2.bitwise_or(threshold, edges)

        morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined = cv2.dilate(combined, morph_kernel, iterations=2)
        combined = cv2.erode(combined, morph_kernel, iterations=1)

        if board_mask is not None:
            combined = cv2.bitwise_and(combined, cv2.bitwise_not(board_mask))

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = frame_bgr.shape[0] * frame_bgr.shape[1]
        max_area = frame_area * float(self.config.max_contour_area_fraction)
        valid = [
            contour
            for contour in contours
            if self.config.min_contour_area < cv2.contourArea(contour) < max_area
        ]
        contour = max(valid, key=cv2.contourArea) if valid else None

        debug_images = {
            "gray": gray,
            "threshold": threshold,
            "edges": edges,
            "combined": combined,
        }
        if not debug:
            debug_images = {}
        return contour, debug_images

    def _project_to_plane(
        self,
        pixel_point: tuple[float, float],
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Back-project one pixel onto the chessboard plane in millimeters."""
        if self.camera_matrix is None:
            return None

        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])

        u, v = pixel_point
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        rotation_inverse = rotation_matrix.T
        ray_direction = rotation_inverse @ np.array([x_norm, y_norm, 1.0])
        camera_origin = rotation_inverse @ (-tvec.flatten())

        if abs(ray_direction[2]) < 1e-6:
            return None

        scale = -camera_origin[2] / ray_direction[2]
        if scale < 0:
            return None

        world_point = camera_origin + scale * ray_direction
        return world_point[:2]

    def _mm_per_pixel(self, rvec: np.ndarray, tvec: np.ndarray) -> float:
        """Estimate local mm-per-pixel scale from one chessboard square."""
        if self.camera_matrix is None:
            return 1.0

        square_points = np.float32(
            [[0.0, 0.0, 0.0], [float(self.config.square_size_mm), 0.0, 0.0]]
        )
        projected, _ = cv2.projectPoints(
            square_points,
            rvec,
            tvec,
            self.camera_matrix,
            np.zeros(5, dtype=np.float32),
        )
        p0 = projected[0][0]
        p1 = projected[1][0]
        pixel_distance = float(np.linalg.norm(p1 - p0))
        if pixel_distance < 1e-6:
            return 1.0
        return float(self.config.square_size_mm) / pixel_distance

    def _measure_dimensions(
        self,
        contour: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> tuple[float, float, float, np.ndarray]:
        """Measure width/length on the plane and estimate height conservatively."""
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)

        plane_points: list[np.ndarray] = []
        for point in box:
            projected = self._project_to_plane((float(point[0]), float(point[1])), rvec, tvec)
            if projected is not None:
                plane_points.append(projected)

        width_mm: float
        length_mm: float
        if len(plane_points) == 4:
            plane = np.array(plane_points, dtype=np.float32)
            side_a = float(np.linalg.norm(plane[1] - plane[0]))
            side_b = float(np.linalg.norm(plane[2] - plane[1]))
            width_mm = min(side_a, side_b)
            length_mm = max(side_a, side_b)
        else:
            mm_per_pixel = self._mm_per_pixel(rvec, tvec)
            side_a, side_b = rect[1]
            width_mm = min(side_a, side_b) * mm_per_pixel
            length_mm = max(side_a, side_b) * mm_per_pixel

        # Single-camera height cannot be exact. This estimate uses contour span
        # and local board scale only as a rough classroom-friendly approximation.
        mm_per_pixel = self._mm_per_pixel(rvec, tvec)
        _, _, _, h_box = cv2.boundingRect(contour)
        estimated_height_mm = float(h_box) * mm_per_pixel * 0.35
        return width_mm, length_mm, estimated_height_mm, np.intp(box)

    def _draw_axes(self, frame_bgr: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, axis_length_mm: float) -> None:
        """Draw board XYZ axes for visual pose feedback."""
        if self.camera_matrix is None:
            return

        axis = np.float32(
            [
                [0, 0, 0],
                [axis_length_mm, 0, 0],
                [0, axis_length_mm, 0],
                [0, 0, -axis_length_mm],
            ]
        )
        projected, _ = cv2.projectPoints(
            axis,
            rvec,
            tvec,
            self.camera_matrix,
            np.zeros(5, dtype=np.float32),
        )
        pts = np.intp(projected.reshape(-1, 2))
        origin = tuple(pts[0])
        cv2.arrowedLine(frame_bgr, origin, tuple(pts[1]), (0, 0, 255), 2, tipLength=0.25)
        cv2.arrowedLine(frame_bgr, origin, tuple(pts[2]), (0, 255, 0), 2, tipLength=0.25)
        cv2.arrowedLine(frame_bgr, origin, tuple(pts[3]), (255, 0, 0), 2, tipLength=0.25)

    def _draw_multiline_text(self, frame_bgr: np.ndarray, lines: list[str], color: tuple[int, int, int]) -> None:
        """Draw readable overlay text with a dark shadow."""
        for index, line in enumerate(lines):
            x = 12
            y = 28 + (index * 28)
            cv2.putText(frame_bgr, line, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame_bgr, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

    def process_frame(self, frame_bgr: np.ndarray, debug: bool = False) -> tuple[Image3DState, dict[str, np.ndarray]]:
        """Process one live frame and return measurement results for the GUI."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            preview = frame_bgr.copy()
            found, corners = self.detect_chessboard(preview)
            if found and corners is not None:
                cv2.drawChessboardCorners(preview, self.config.chessboard_size, corners, found)
            self._draw_multiline_text(
                preview,
                [
                    "Calibration required before Image 3D measurement.",
                    f"Show board {self.config.chessboard_cols}x{self.config.chessboard_rows} and capture samples.",
                ],
                (0, 180, 255),
            )
            state = Image3DState(
                frame_bgr=preview,
                board_visible=found,
                object_visible=False,
                message="Calibration data missing.",
                rms_error=self.rms_error,
                calibration_samples=len(self.image_points),
            )
            return state, {}

        undistorted = self.undistort(frame_bgr)
        display = undistorted.copy()
        found, corners = self.detect_chessboard(undistorted)

        if not found or corners is None:
            self._draw_multiline_text(
                display,
                [
                    "Chessboard not visible.",
                    "Measurement needs the reference plane in view.",
                ],
                (0, 0, 255),
            )
            state = Image3DState(
                frame_bgr=display,
                board_visible=False,
                object_visible=False,
                message="Chessboard not visible.",
                rms_error=self.rms_error,
                calibration_samples=len(self.image_points),
            )
            return state, {}

        cv2.drawChessboardCorners(display, self.config.chessboard_size, corners, True)
        pose_ok, rvec, tvec = self._estimate_pose(corners)
        if not pose_ok or rvec is None or tvec is None:
            self._draw_multiline_text(display, ["Pose estimation failed."], (0, 0, 255))
            state = Image3DState(
                frame_bgr=display,
                board_visible=True,
                object_visible=False,
                message="Pose estimation failed.",
                rms_error=self.rms_error,
                calibration_samples=len(self.image_points),
            )
            return state, {}

        self._draw_axes(display, rvec, tvec, axis_length_mm=self.config.square_size_mm * 3.0)
        board_mask = self._board_mask(undistorted.shape, corners)
        contour, debug_images = self._detect_object_contour(undistorted, board_mask, debug)

        if contour is None:
            self._draw_multiline_text(
                display,
                [
                    "No object contour found.",
                    "Place the object on or next to the board with good contrast.",
                ],
                (0, 215, 255),
            )
            state = Image3DState(
                frame_bgr=display,
                board_visible=True,
                object_visible=False,
                message="No object detected.",
                rms_error=self.rms_error,
                calibration_samples=len(self.image_points),
            )
            return state, debug_images

        width_mm, length_mm, estimated_height_mm, box = self._measure_dimensions(contour, rvec, tvec)
        cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
        cv2.drawContours(display, [box], -1, (0, 255, 255), 2)

        moments = cv2.moments(contour)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
        else:
            rect = cv2.minAreaRect(contour)
            cx, cy = int(rect[0][0]), int(rect[0][1])

        label_lines = [
            f"Width: {width_mm:.1f} mm",
            f"Length: {length_mm:.1f} mm",
            f"Height: ~{estimated_height_mm:.1f} mm (estimate)",
        ]
        for idx, line in enumerate(label_lines):
            cv2.putText(display, line, (cx - 80, cy - 18 + (idx * 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(display, line, (cx - 80, cy - 18 + (idx * 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

        info_lines = [
            f"RMS: {self.rms_error:.4f} px" if self.rms_error is not None else "RMS: N/A",
            "Width/length are on-plane values.",
            "Height is approximate with one RGB camera.",
        ]
        self._draw_multiline_text(display, info_lines, (255, 255, 255))

        state = Image3DState(
            frame_bgr=display,
            board_visible=True,
            object_visible=True,
            message="Measurement updated.",
            width_mm=float(width_mm),
            length_mm=float(length_mm),
            estimated_height_mm=float(estimated_height_mm),
            rms_error=self.rms_error,
            calibration_samples=len(self.image_points),
        )
        return state, debug_images