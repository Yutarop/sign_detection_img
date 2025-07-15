import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__("signboard_detector")
        self.bridge = CvBridge()
        
        package_share_dir = get_package_share_directory("sign_detection_img")
        model_path = os.path.join(package_share_dir, "models", "sign_detection.pt")
        self.model = YOLO(model_path)
        
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        
        cv2.namedWindow("Sign Detection", cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info("YOLOv8 with real-time display has started.")
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        
        results = self.model(frame)
        
        annotated_frame = frame.copy()
        detected = False
        sign_count = 0
        
        for box in results[0].boxes:
            if box is not None:
                cls_id = int(box.cls[0].item())
                cls_name = self.model.names[cls_id]
                confidence = float(box.conf[0].item())
                
                if "signboard" in cls_name.lower():
                    detected = True
                    sign_count += 1
                    
                    x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                    
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    label = f"signboard: {confidence:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    
                    cv2.rectangle(annotated_frame, 
                                (x1, y1 - label_size[1] - 10), 
                                (x1 + label_size[0], y1), 
                                (0, 255, 0), -1)
                    
                    cv2.putText(annotated_frame, label, 
                              (x1, y1 - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        if detected:
            self.get_logger().info(f"Signboard detected! Count: {sign_count}")
        else:
            self.get_logger().info("No Signboard found.")
        
        cv2.imshow("Signboard Detection", annotated_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"): 
            self.get_logger().info("ESC pressed. Shutting down...")
            self.destroy_node()
            rclpy.shutdown()
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()