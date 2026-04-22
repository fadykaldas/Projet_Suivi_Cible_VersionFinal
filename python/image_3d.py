from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class Image3DConfig:
    chessboard_cols: int = 9
    chessboard_rows: int = 6
    square_size_mm: float = 25.0
    min_calibration_frames: int = 12
    blur_kernel_size: int = 5
    min_contour_area: int = 2000


@dataclass
class Image3DState:
    frame_bgr: np.ndarray
    width_mm: Optional[float] = None
    length_mm: Optional[float] = None
    estimated_height_mm: Optional[float] = None
    rms_error: Optional[float] = None
    calibration_samples: int = 0
    x_board_mm: Optional[float] = None
    y_board_mm: Optional[float] = None
    message: str = "Waiting..."


class SingleCameraChessboardMeasurer:
    def __init__(self, config: Image3DConfig, calibration_file: Path | str):
        self.config = config
        self.calibration_file = Path(calibration_file)

        self.image_points: list[np.ndarray] = []
        self.object_points: list[np.ndarray] = []

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.rvecs = []
        self.tvecs = []
        self.rms_error: Optional[float] = None

        self.board_size = (int(self.config.chessboard_cols), int(self.config.chessboard_rows))
        self.objp = self._build_chessboard_object_points()

        # Board lock / memory
        self.last_corners: Optional[np.ndarray] = None
        self.last_H: Optional[np.ndarray] = None
        self.board_locked: bool = False
        self.frames_since_seen: int = 0
        self.max_frames_without_board = 45

        self._load_saved_calibration()

    # ------------------------------------------------------------------
    # PUBLIC API
    # ------------------------------------------------------------------
    def capture_calibration_sample(self, frame_bgr: np.ndarray):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = self._find_chessboard(gray)

        if not found or corners is None:
            return False, "Chessboard not found. Show the full board clearly."

        corners = self._refine_corners(gray, corners)

        self.image_points.append(corners)
        self.object_points.append(self.objp.copy())

        count = len(self.image_points)
        need = int(self.config.min_calibration_frames)

        if count >= need:
            return True, f"Calibration sample saved ({count}/{need} minimum). Ready to calibrate."
        return True, f"Calibration sample saved ({count}/{need} minimum)."

    def calibrate(self):
        if len(self.image_points) < int(self.config.min_calibration_frames):
            return False, (
                f"Need at least {self.config.min_calibration_frames} chessboard views. "
                f"Current: {len(self.image_points)}."
            )

        img_points = [pts.astype(np.float32) for pts in self.image_points]
        obj_points = [pts.astype(np.float32) for pts in self.object_points]

        # Taille image estimée depuis le dernier sample
        last = img_points[-1]
        max_x = int(np.max(last[:, 0, 0])) + 50
        max_y = int(np.max(last[:, 0, 1])) + 50
        image_size = (max(max_x, 640), max(max_y, 480))

        try:
            rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                obj_points,
                img_points,
                image_size,
                None,
                None
            )
        except Exception as e:
            return False, f"Calibration failed: {e}"

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rvecs = rvecs
        self.tvecs = tvecs
        self.rms_error = float(rms)

        self._save_calibration()
        return True, f"Calibration successful. RMS error: {self.rms_error:.4f} px"

    def reset_calibration_samples(self):
        self.image_points.clear()
        self.object_points.clear()

    def delete_saved_calibration(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = []
        self.tvecs = []
        self.rms_error = None

        if self.calibration_file.exists():
            try:
                self.calibration_file.unlink()
            except Exception:
                pass

    def process_frame(self, frame_bgr: np.ndarray, debug: bool = False):
        frame = frame_bgr.copy()
        debug_images = {}

        state = Image3DState(
            frame_bgr=frame,
            rms_error=self.rms_error,
            calibration_samples=len(self.image_points),
            message="Show the chessboard to lock the board reference."
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1) Detect chessboard if visible
        found, corners = self._find_chessboard(gray)

        H = None
        active_corners = None
        board_status_text = ""

        if found and corners is not None:
            corners = self._refine_corners(gray, corners)
            H = self._compute_homography_from_corners(corners)

            if H is not None:
                self.last_corners = corners.copy()
                self.last_H = H.copy()
                self.board_locked = True
                self.frames_since_seen = 0
                active_corners = corners
                board_status_text = "BOARD LOCKED (live)"
        else:
            self.frames_since_seen += 1

            # Reuse last locked board even if partially hidden
            if self.board_locked and self.last_H is not None and self.frames_since_seen <= self.max_frames_without_board:
                H = self.last_H.copy()
                active_corners = self.last_corners.copy() if self.last_corners is not None else None
                board_status_text = "BOARD LOCKED (memory)"
            else:
                self.board_locked = False
                self.last_H = None
                self.last_corners = None

        # 2) If no board reference at all, stop here
        if H is None:
            cv2.putText(
                frame,
                "Show full chessboard to lock reference",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )
            state.frame_bgr = frame
            state.message = "Chessboard not locked yet."
            if debug:
                debug_images["threshold"] = gray
                debug_images["combined"] = frame.copy()
            return state, debug_images

        # 3) Draw board if corners currently known
        if active_corners is not None:
            try:
                cv2.drawChessboardCorners(frame, self.board_size, active_corners, True)
            except Exception:
                pass

        board_polygon = self._board_polygon_from_homography(H)
        cv2.polylines(frame, [board_polygon], True, (0, 255, 255), 2)

        # 4) Detect object INSIDE the board reference
        threshold_img, combined_debug, contour = self._detect_object_using_board_reference(frame_bgr, H, board_polygon)

        if debug:
            debug_images["threshold"] = threshold_img
            debug_images["combined"] = combined_debug

        cv2.putText(
            frame,
            board_status_text,
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2
        )

        if contour is None:
            cv2.putText(
                frame,
                "No object found on board",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 140, 255),
                2
            )
            state.frame_bgr = frame
            state.message = "Board locked. Place object on the chessboard."
            return state, debug_images

        # 5) Measure object in board coordinates
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect).astype(np.float32)
        box_int = np.int32(box)
        cv2.drawContours(frame, [box_int], 0, (0, 255, 0), 2)

        board_box_pts = cv2.perspectiveTransform(
            box.reshape(-1, 1, 2),
            H
        ).reshape(-1, 2)

        edges = []
        for i in range(4):
            p1 = board_box_pts[i]
            p2 = board_box_pts[(i + 1) % 4]
            edges.append(float(np.linalg.norm(p2 - p1)))

        edges.sort()
        state.width_mm = float(np.mean(edges[:2]))
        state.length_mm = float(np.mean(edges[2:]))

        # Use bottom-center of object as position reference
        bottom_idx = np.argsort(box[:, 1])[-2:]
        bottom_pts = box[bottom_idx]
        bottom_center = np.mean(bottom_pts, axis=0).astype(np.float32)

        obj_board_pt = cv2.perspectiveTransform(
            bottom_center.reshape(1, 1, 2),
            H
        )[0, 0]

        state.x_board_mm = float(obj_board_pt[0])
        state.y_board_mm = float(obj_board_pt[1])

        # Clamp only for sanity
        state.x_board_mm = float(max(-1000.0, min(5000.0, state.x_board_mm)))
        state.y_board_mm = float(max(-1000.0, min(5000.0, state.y_board_mm)))

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            state.estimated_height_mm = self._estimate_height_mm(rect)
        else:
            state.estimated_height_mm = None

        center_int = tuple(np.int32(bottom_center))
        cv2.circle(frame, center_int, 6, (0, 255, 255), -1)

        cv2.putText(
            frame,
            f"X: {state.x_board_mm:.1f} mm   Y: {state.y_board_mm:.1f} mm",
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )
        cv2.putText(
            frame,
            f"Width: {state.width_mm:.1f} mm",
            (20, 175),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )
        cv2.putText(
            frame,
            f"Length: {state.length_mm:.1f} mm",
            (20, 210),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        if state.estimated_height_mm is not None:
            cv2.putText(
                frame,
                f"Est. height: {state.estimated_height_mm:.1f} mm",
                (20, 245),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )
            state.message = "Board locked. Object coordinates measured."
        else:
            cv2.putText(
                frame,
                "Calibration optional for better height estimate",
                (20, 245),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2
            )
            state.message = "Board locked. Object coordinates measured."

        state.frame_bgr = frame
        return state, debug_images

    # ------------------------------------------------------------------
    # INTERNALS
    # ------------------------------------------------------------------
    def _build_chessboard_object_points(self):
        cols, rows = self.board_size
        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp[:, :2] *= float(self.config.square_size_mm)
        return objp

    def _find_chessboard(self, gray: np.ndarray):
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, self.board_size, flags)
        return found, corners

    def _refine_corners(self, gray: np.ndarray, corners: np.ndarray):
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )
        return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    def _compute_homography_from_corners(self, corners: np.ndarray):
        try:
            img_pts = corners.reshape(-1, 2).astype(np.float32)
            board_pts_mm = self.objp[:, :2].astype(np.float32)
            H, _ = cv2.findHomography(img_pts, board_pts_mm)
            return H
        except Exception:
            return None

    def _board_polygon_from_homography(self, H: np.ndarray):
        cols, rows = self.board_size
        sq = float(self.config.square_size_mm)

        width_mm = (cols - 1) * sq
        height_mm = (rows - 1) * sq

        board_corners_mm = np.array([
            [0.0, 0.0],
            [width_mm, 0.0],
            [width_mm, height_mm],
            [0.0, height_mm]
        ], dtype=np.float32).reshape(-1, 1, 2)

        H_inv = np.linalg.inv(H)
        polygon_img = cv2.perspectiveTransform(board_corners_mm, H_inv).reshape(-1, 2)
        return np.int32(polygon_img)

    def _detect_object_using_board_reference(self, frame_bgr: np.ndarray, H: np.ndarray, board_polygon: np.ndarray):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        k = int(self.config.blur_kernel_size)
        if k % 2 == 0:
            k += 1
        if k < 3:
            k = 3

        blur = cv2.GaussianBlur(gray, (k, k), 0)

        # Only search inside board area
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, board_polygon, 255)

        # Warp image to top-down board view
        cols, rows = self.board_size
        sq = float(self.config.square_size_mm)
        board_w = int((cols - 1) * sq)
        board_h = int((rows - 1) * sq)

        board_w = max(board_w, 100)
        board_h = max(board_h, 100)

        topdown = cv2.warpPerspective(frame_bgr, H, (board_w, board_h))
        topdown_gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)

        # Threshold in top-down view
        topdown_blur = cv2.GaussianBlur(topdown_gray, (k, k), 0)
        thresh = cv2.adaptiveThreshold(
            topdown_blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            8
        )

        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        total_area = float(board_w * board_h)

        for c in contours:
            area = cv2.contourArea(c)
            if area < float(self.config.min_contour_area):
                continue

            x, y, w, h = cv2.boundingRect(c)

            # Ignore almost full board
            if area > total_area * 0.85:
                continue

            if w < 15 or h < 15:
                continue

            if area > best_area:
                best = c
                best_area = area

        debug_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

        if best is None:
            return thresh, debug_bgr, None

        cv2.drawContours(debug_bgr, [best], -1, (0, 255, 0), 2)

        # Convert contour from top-down board image back to original camera image
        H_inv = np.linalg.inv(H)
        contour_float = best.astype(np.float32)
        contour_img = cv2.perspectiveTransform(contour_float, H_inv)

        return thresh, debug_bgr, contour_img.astype(np.int32)

    def _estimate_height_mm(self, rect):
        (_, _), (w, h), _ = rect
        minor_px = min(w, h)

        if minor_px <= 0:
            return None

        est = 0.08 * float(minor_px)
        fx = float(self.camera_matrix[0, 0]) if self.camera_matrix is not None else 800.0
        scale = 500.0 / max(fx, 1.0)
        est_mm = est * scale * 10.0

        return float(max(1.0, min(est_mm, 200.0)))

    def _save_calibration(self):
        try:
            self.calibration_file.parent.mkdir(parents=True, exist_ok=True)
            np.savez(
                str(self.calibration_file),
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
                rms_error=np.array([self.rms_error if self.rms_error is not None else -1.0], dtype=np.float64),
            )
        except Exception:
            pass

    def _load_saved_calibration(self):
        if not self.calibration_file.exists():
            return

        try:
            data = np.load(str(self.calibration_file), allow_pickle=True)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            rms = data["rms_error"]
            if rms is not None and len(rms) > 0 and float(rms[0]) >= 0:
                self.rms_error = float(rms[0])
        except Exception:
            self.camera_matrix = None
            self.dist_coeffs = None
            self.rms_error = None 