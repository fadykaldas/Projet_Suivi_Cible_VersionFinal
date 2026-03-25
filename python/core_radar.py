"""
Radar display and data management.
Handles real-time radar updates and visualization.
"""
import time
import math
from collections import deque
from config import AppConfig


class RadarDataManager:
    """Manages radar data and state"""
    
    def __init__(self, config: AppConfig):
        self.config = config
        self.last_angle = 90.0
        self.last_distance = None
        self.points = deque(maxlen=600)  # (angle, dist, timestamp)
        self.sweep_angles = deque(maxlen=config.sweep_trail)
    
    def update_config(self, config: AppConfig) -> None:
        """Update configuration"""
        self.config = config
        self.sweep_angles = deque(self.sweep_angles, maxlen=config.sweep_trail)
    
    def map_angle(self, angle_deg: float) -> float:
        """
        Map raw servo angle to display angle with calibration.
        
        Applies: servo limits, offset, and flip
        """
        a = max(self.config.servo_min_angle, 
                min(self.config.servo_max_angle, float(angle_deg)))
        
        if self.config.servo_max_angle != self.config.servo_min_angle:
            a = (a - self.config.servo_min_angle) * 180.0 / (self.config.servo_max_angle - self.config.servo_min_angle)
        
        a = a + self.config.radar_offset_deg
        a = max(0.0, min(180.0, a))
        
        if self.config.radar_flip:
            a = 180.0 - a
        
        return a
    
    def update_data(self, angle_raw: float = None, dist_cm: float = None) -> None:
        """
        Update radar with new angle and distance data.
        
        Args:
            angle_raw: Raw servo angle (before calibration)
            dist_cm: Distance in centimeters
        """
        now = time.time()
        
        if angle_raw is not None:
            self.last_angle = self.map_angle(angle_raw)
            self.sweep_angles.appendleft(self.last_angle)
        else:
            self.sweep_angles.appendleft(self.last_angle)
        
        if dist_cm is not None:
            self.last_distance = float(dist_cm)
            if 0.0 < self.last_distance <= self.config.radar_max_range_cm:
                self.points.append((self.last_angle, self.last_distance, now))
    
    def get_stale_age(self) -> float:
        """Get time in seconds before a point becomes stale"""
        return self.config.point_ttl_sec
    
    def get_sweep_data(self) -> tuple:
        """
        Get current sweep and point data.
        
        Returns:
            (sweep_angles, points_list, last_angle, last_distance)
        """
        now = time.time()
        while self.points and (now - self.points[0][2]) > self.config.point_ttl_sec:
            self.points.popleft()
        
        return (
            list(self.sweep_angles),
            list(self.points),
            self.last_angle,
            self.last_distance
        )


class RadarState:
    """Manages camera automation based on radar detection"""
    
    def __init__(self, config: AppConfig):
        self.config = config
        self.state = "idle"  # "idle" or "open"
        self.detect_start = None
        self.loss_start = None
    
    def reset(self) -> None:
        """Reset state"""
        self.state = "idle"
        self.detect_start = None
        self.loss_start = None
    
    def update(self, distance: float = None) -> str:
        """
        Update state machine. Returns action: None, "open_camera", or "close_camera"
        
        Args:
            distance: Current distance from radar (None if unknown)
        
        Returns:
            Action to take: None, "open_camera", or "close_camera"
        """
        now = time.time()
        detected = (distance is not None) and (distance <= self.config.on_threshold_cm)
        lost = (distance is None) or (distance >= self.config.off_threshold_cm)
        
        if self.state == "idle":
            if detected:
                if self.detect_start is None:
                    self.detect_start = now
                elif (now - self.detect_start) >= self.config.trigger_confirm_sec:
                    self.state = "open"
                    self.detect_start = None
                    self.loss_start = None
                    return "open_camera"
            else:
                self.detect_start = None
        
        elif self.state == "open":
            if lost:
                if self.loss_start is None:
                    self.loss_start = now
                elif (now - self.loss_start) >= self.config.cam_close_after_radar_loss:
                    self.state = "idle"
                    self.detect_start = None
                    self.loss_start = None
                    return "close_camera"
            else:
                self.loss_start = None
        
        return None
