import cv2
import depthai as dai
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class BallTracker:
    def __init__(self, history_size=10):
        self.positions = deque(maxlen=history_size)
        self.velocities = deque(maxlen=history_size-1)
        self.last_detection = None
        self.frames_since_detection = 0
        
    def update(self, frame):
        h, w = frame.shape[:2]
        predicted_pos = self.predict_position()
        
        # Define search regions
        if predicted_pos is not None:
            # If we have a prediction, search near it first
            px, py = predicted_pos
            search_regions = [
                (px - 100, py - 100, px + 100, py + 100),  # Predicted region
                (0, 0, w, h)  # Full frame
            ]
        else:
            search_regions = [(0, 0, w, h)]  # Just search full frame
            
        ball_detected = None
        
        for x1, y1, x2, y2 in search_regions:
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(w, int(x2)), min(h, int(y2))
            
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                continue
                
            # Try different parameters based on detection history
            param2_values = [30, 25, 20] if self.frames_since_detection < 5 else [20, 15, 10]
            
            for param2 in param2_values:
                circles = self.detect_ball(roi, param2=param2)
                if circles is not None:
                    # Adjust coordinates back to full frame
                    x, y, r = circles
                    ball_detected = (x + x1, y + y1, r)
                    break
            
            if ball_detected is not None:
                break
        
        if ball_detected is not None:
            x, y, r = ball_detected
            self.positions.append((x, y))
            
            if len(self.positions) >= 2:
                last_x, last_y = self.positions[-2]
                vx = x - last_x
                vy = y - last_y
                self.velocities.append((vx, vy))
            
            self.last_detection = ball_detected
            self.frames_since_detection = 0
        else:
            self.frames_since_detection += 1
            
        return ball_detected
    
    def detect_ball(self, frame, param2=30):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,
            param2=param2,
            minRadius=20,
            maxRadius=100
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            x, y, r = circles[0][0]
            return (x, y, r)
        
        return None
    
    def predict_position(self):
        if len(self.positions) < 2:
            return None
            
        if self.frames_since_detection > 10:
            return None
            
        if self.velocities:
            avg_vx = sum(v[0] for v in self.velocities) / len(self.velocities)
            avg_vy = sum(v[1] for v in self.velocities) / len(self.velocities)
            
            last_x, last_y = self.positions[-1]
            predicted_x = last_x + avg_vx * (self.frames_since_detection + 1)
            predicted_y = last_y + avg_vy * (self.frames_since_detection + 1)
            
            return (predicted_x, predicted_y)
        
        return None

class BallTrackingNode(Node):
    def __init__(self):
        super().__init__('ball_tracking_node')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.spatialLocationCalculator = self.pipeline.create(dai.node.SpatialLocationCalculator)

        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        self.xoutSpatialData = self.pipeline.create(dai.node.XLinkOut)
        self.xinSpatialCalcConfig = self.pipeline.create(dai.node.XLinkIn)

        self.xoutRgb.setStreamName("rgb")
        self.xoutDepth.setStreamName("depth")
        self.xoutSpatialData.setStreamName("spatialData")
        self.xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        # Properties
        self.camRgb.setPreviewSize(640, 480)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setCamera("left")
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setCamera("right")

        # Configure stereo depth
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(True)

        # Configure initial spatial calculator settings
        self.config = dai.SpatialLocationCalculatorConfigData()
        self.config.depthThresholds.lowerThreshold = 100
        self.config.depthThresholds.upperThreshold = 10000
        self.config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN

        # Initial ROI in center of frame
        self.topLeft = dai.Point2f(0.4, 0.4)
        self.bottomRight = dai.Point2f(0.6, 0.6)
        self.config.roi = dai.Rect(self.topLeft, self.bottomRight)
        self.spatialLocationCalculator.initialConfig.addROI(self.config)

        # Linking
        self.camRgb.preview.link(self.xoutRgb.input)
        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)
        self.stereo.depth.link(self.spatialLocationCalculator.inputDepth)
        self.spatialLocationCalculator.passthroughDepth.link(self.xoutDepth.input)
        self.spatialLocationCalculator.out.link(self.xoutSpatialData.input)
        self.xinSpatialCalcConfig.out.link(self.spatialLocationCalculator.inputConfig)

        # Initialize tracker
        self.tracker = BallTracker()

        # Initialize device and get queues
        self.device = dai.Device(self.pipeline)
        self.rgbQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
        self.spatialCalcConfigInQueue = self.device.getInputQueue("spatialCalcConfig")

        # Create timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # 30 FPS

#    def calculate_steering_angle(self, y, z):
#        angle = math.atan2(y, z)
#        normalized_angle = angle / (math.pi/2)
#        return normalized_angle

    def process_frame(self):
        rgbFrame = self.rgbQueue.get().getCvFrame()
        ball = self.tracker.update(rgbFrame)
        
        if ball is not None:
            x, y, r = ball
            
            # Convert y to camera-centered coordinates
            y_centered = y - (rgbFrame.shape[0] / 2)
            
            # Calculate normalized coordinates for ROI
            x_norm = x / rgbFrame.shape[1]
            y_norm = y / rgbFrame.shape[0]
            
            # Create ROI around ball center
            roi_size = 0.1
            topLeft = dai.Point2f(
                max(0, x_norm - roi_size/2),
                max(0, y_norm - roi_size/2)
            )
            bottomRight = dai.Point2f(
                min(1, x_norm + roi_size/2),
                min(1, y_norm + roi_size/2)
            )

            # Update spatial calculator config
            self.config.roi = dai.Rect(topLeft, bottomRight)
            cfg = dai.SpatialLocationCalculatorConfig()
            cfg.addROI(self.config)
            self.spatialCalcConfigInQueue.send(cfg)


            # Get spatial data
            spatialData = self.spatialCalcQueue.get().getSpatialLocations()
            if spatialData:
                for depthData in spatialData:
                    z = int(depthData.spatialCoordinates.z)
                    
                    # Calculate steering angle
                    steering_angle = (x-320)/320
                    
                    # Create and publish Twist message
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.linear.z = 0.0 # float(z)  # depth in z
                    cmd.angular.x = 0.0
                    cmd.angular.y = 0.0
                    cmd.angular.z = float(steering_angle)  # steering in angular z
                    self.cmd_vel_publisher.publish(cmd)
                    
                    self.get_logger().info(f'Steering angle: {steering_angle:.2f}, Depth: {z}mm')
            
            # Draw visualizations
            cv2.circle(rgbFrame, (int(x), int(y)), int(r), (0, 255, 0), 2)
            cv2.circle(rgbFrame, (int(x), int(y)), 2, (0, 0, 255), 3)
            if spatialData:
                cv2.putText(rgbFrame, f"Depth: {z}mm, Steering angle: {steering_angle}", 
                           (int(x - r), int(y - r - 10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show frame
        cv2.imshow("Ball Tracking", rgbFrame)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()
        if hasattr(self, 'device'):
            self.device.close()

def main(args=None):
    rclpy.init(args=args)
    node = BallTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
