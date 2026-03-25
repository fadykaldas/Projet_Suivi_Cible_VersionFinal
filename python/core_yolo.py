"""
YOLO object detection module.
Handles YOLOv8 model loading and inference.
"""
import cv2

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YOLODetector:
    """Manages YOLOv8 object detection"""
    
    def __init__(self):
        self.model = None
        self.camera = None
        self.target_object = "phone"
        self.confidence_threshold = 0.3
    
    @staticmethod
    def is_available() -> bool:
        """Check if ultralytics module is installed"""
        return YOLO_AVAILABLE
    
    def load_model(self, model_name: str = "yolov8m.pt") -> bool:
        """
        Load a YOLO model.
        
        Args:
            model_name: Model file (e.g., "yolov8m.pt", "yolov8n.pt")
        
        Returns:
            True if model loaded successfully
        """
        if not YOLO_AVAILABLE:
            raise RuntimeError("YOLOv8 not installed. Run: pip install ultralytics")
        
        try:
            print(f"Loading YOLO model: {model_name}...")
            self.model = YOLO(model_name)
            available_classes = list(self.model.names.values())
            print(f"✓ Model loaded. Available classes: {', '.join(available_classes)}")
            return True
        except Exception as e:
            raise Exception(f"Cannot load YOLO model: {str(e)}")
    
    def open_camera(self, cam_index: int = 0) -> bool:
        """
        Open camera for YOLO detection.
        
        Args:
            cam_index: Camera device index
        
        Returns:
            True if camera opened successfully
        """
        try:
            self.camera = cv2.VideoCapture(int(cam_index))
            if not self.camera.isOpened():
                raise Exception(f"Camera at index {cam_index} not accessible")
            return True
        except Exception as e:
            raise Exception(str(e))
    
    def close_camera(self) -> None:
        """Close camera"""
        if self.camera:
            try:
                self.camera.release()
            except Exception:
                pass
            self.camera = None
    
    def is_ready(self) -> bool:
        """Check if detector is ready (model loaded and camera open)"""
        return self.model is not None and self.camera is not None and self.camera.isOpened()
    
    def detect_frame(self, frame=None) -> dict:
        """
        Run YOLO inference on a frame.
        
        Args:
            frame: Optional frame. If None, captures from camera.
        
        Returns:
            Dictionary with detection results:
            {
                'detected': bool,
                'object_name': str,
                'confidence': float,
                'frame': annotated_frame,
                'all_detections': [(label, conf), ...]
            }
        """
        if not self.is_ready():
            return {'detected': False, 'object_name': None, 'confidence': 0.0, 'frame': None, 'all_detections': []}
        
        try:
            # Capture frame if not provided
            if frame is None:
                ret, frame = self.camera.read()
                if not ret:
                    return {'detected': False, 'object_name': None, 'confidence': 0.0, 'frame': None, 'all_detections': []}
            
            # Run inference
            results = self.model(frame, verbose=False, conf=self.confidence_threshold)
            detections = results[0]
            
            # Collect all detections
            all_detections = []
            target_detected = False
            best_confidence = 0.0
            best_label = None
            
            for box in detections.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                label = self.model.names[class_id]
                all_detections.append((label, confidence))
                
                # Check for target object
                if label.lower() == self.target_object.lower():
                    if confidence > best_confidence:
                        target_detected = True
                        best_confidence = confidence
                        best_label = label
            
            # Annotate frame
            annotated_frame = detections.plot()
            
            return {
                'detected': target_detected,
                'object_name': best_label,
                'confidence': best_confidence,
                'frame': annotated_frame,
                'all_detections': all_detections
            }
        
        except Exception as e:
            print(f"YOLO detection error: {e}")
            return {'detected': False, 'object_name': None, 'confidence': 0.0, 'frame': None, 'all_detections': []}
    
    def cleanup(self) -> None:
        """Clean up resources"""
        self.close_camera()
        self.model = None
