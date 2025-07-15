import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class RobustLaneDetector(Node):
    def __init__(self):
        super().__init__('robust_lane_detector')
        self.bridge = CvBridge()
        
        # Parameters (tunable)
        self.declare_parameter('roi_height_ratio', 0.6)
        self.declare_parameter('roi_width_ratio', 0.8)
        self.declare_parameter('min_line_angle', 20.0)
        self.declare_parameter('lane_width_pixels', 300)  # Estimated lane width
        
        self.subscription = self.create_subscription(
            Image,
            '/camera_rgb/image_raw',
            self.image_callback,
            10)
            
        self.offset_pub = self.create_publisher(Float32, '/lane_offset', 10)
        self.last_known_lane_width = None
        self.last_known_offset = 0.0
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = frame.shape[:2]
            
            # Get parameters
            roi_height_ratio = self.get_parameter('roi_height_ratio').value
            roi_width_ratio = self.get_parameter('roi_width_ratio').value
            
            # ROI with perspective adjustment
            roi_start_y = int(height * roi_height_ratio)
            roi_end_y = height
            roi_width = int(width * roi_width_ratio)
            roi_start_x = (width - roi_width) // 2
            roi_end_x = roi_start_x + roi_width
            roi = frame[roi_start_y:roi_end_y, roi_start_x:roi_end_x]
            
            # Preprocessing
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (9, 9), 0)
            edges = cv2.Canny(blur, 50, 150)
            
            # Detect lines
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 
                                  minLineLength=40, maxLineGap=30)
            
            left_lines, right_lines = [], []
            line_img = roi.copy()
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.degrees(np.arctan2(y2-y1, x2-x1))
                    min_angle = self.get_parameter('min_line_angle').value
                    
                    if abs(angle) > min_angle:
                        # Calculate line properties
                        slope = (y2-y1)/(x2-x1) if (x2-x1) != 0 else 1000
                        intercept = y1 - slope*x1
                        line_length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                        
                        # Classify left/right lines
                        if slope < 0 and x1 < width/2 and x2 < width/2:
                            left_lines.append((slope, intercept, line_length))
                        elif slope > 0 and x1 > width/2 and x2 > width/2:
                            right_lines.append((slope, intercept, line_length))
                            
                        cv2.line(line_img, (x1,y1), (x2,y2), (255,0,255), 2)
            
            # Calculate lane center and offset
            image_center = roi.shape[1] / 2
            center_offset = self.last_known_offset  # Default to last known
            
            if left_lines and right_lines:
                # Both lines detected - normal case
                left_avg = np.average([l[0] for l in left_lines], weights=[l[2] for l in left_lines])
                left_int = np.average([l[1] for l in left_lines], weights=[l[2] for l in left_lines])
                right_avg = np.average([r[0] for r in right_lines], weights=[r[2] for r in right_lines])
                right_int = np.average([r[1] for r in right_lines], weights=[r[2] for r in right_lines])
                
                # Calculate intersection points at bottom of image
                y = roi.shape[0]
                left_x = int((y - left_int) / left_avg) if left_avg != 0 else 0
                right_x = int((y - right_int) / right_avg) if right_avg != 0 else roi.shape[1]
                
                lane_center = (left_x + right_x) / 2
                center_offset = (image_center - lane_center) / (roi.shape[1] / 2)
                
                # Update lane width estimate
                self.last_known_lane_width = abs(right_x - left_x)
                cv2.line(line_img, (left_x, y), (left_x, y-50), (0,255,0), 3)
                cv2.line(line_img, (right_x, y), (right_x, y-50), (0,255,0), 3)
                
            elif left_lines or right_lines:
                # Only one line detected - use historical lane width
                if self.last_known_lane_width is not None:
                    if left_lines:
                        left_avg = np.average([l[0] for l in left_lines], weights=[l[2] for l in left_lines])
                        left_int = np.average([l[1] for l in left_lines], weights=[l[2] for l in left_lines])
                        left_x = int((roi.shape[0] - left_int) / left_avg)
                        lane_center = left_x + self.last_known_lane_width/2
                    else:
                        right_avg = np.average([r[0] for r in right_lines], weights=[r[2] for r in right_lines])
                        right_int = np.average([r[1] for r in right_lines], weights=[r[2] for r in right_lines])
                        right_x = int((roi.shape[0] - right_int) / right_avg)
                        lane_center = right_x - self.last_known_lane_width/2
                    
                    center_offset = (image_center - lane_center) / (roi.shape[1] / 2)
            
            # Apply low-pass filter to offset
            self.last_known_offset = 0.8*self.last_known_offset + 0.2*center_offset
            
            # Publish filtered offset
            self.offset_pub.publish(Float32(data=self.last_known_offset))
            
            # Visualization
            frame[roi_start_y:roi_end_y, roi_start_x:roi_end_x] = line_img
            cv2.putText(frame, f"Offset: {self.last_known_offset:.2f}", (30, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.imshow("Enhanced Lane Detection", frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RobustLaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()