#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        
        # Create ROS2 publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Variables to store last known values
        self.last_depth = 0.0
        self.last_angle = 0.0
        self.ball_detected = False
        self.ball_moved = False
        
        # New variables for robust movement detection
        self.initial_depth = None  # Initial depth when ball is first detected
        self.consecutive_closer_frames = 0  # Counter for frames where ball is closer
        self.frames_required = 2  # Number of consecutive frames required to trigger movement
        self.movement_tolerance = 10  # Movement detection tolerance in mm
        
        # Safety-related variables
        self.throttle_start_time = None
        self.MAX_THROTTLE_DURATION = 0.75  # Maximum duration in seconds
        self.emergency_stop = False
        self.last_throttle_command = time.time()
        self.safety_timeout = 0.5  # Stop if no new commands received in 0.5 seconds
	        
        # Get model path
        self.nnPath = str((Path(__file__).parent / Path('/home/projects/depthai-python/examples/models/yolov8n_coco_640x352.blob')).resolve().absolute())
        if not Path(self.nnPath).exists():
            raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

        # Initialize DepthAI pipeline
        self.initialize_pipeline()
        
        # Start safety timer
        self.create_timer(0.1, self.safety_check)  # Check every 100ms
    
    
    def initialize_pipeline(self):
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        nnOut = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        nnOut.setStreamName("nn")
        xoutDepth.setStreamName("depth")

        # Enable RGBR compression to reduce USB bandwidth
        camRgb.setPreviewSize(640, 352)  # Keep resolution modest
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(35)
        
        # Enable RGB ISP (Image Signal Processor) for better image quality
        camRgb.setIspScale(2, 3)  # Downscale RGB to reduce ISP load
        
        # Configure camera to minimize latency
        camRgb.setPreviewKeepAspectRatio(False)  # Allow scaling without maintaining aspect ratio
        
        # MonoCamera settings - use lower resolution for depth
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setFps(35)  # Higher FPS for more accurate depth
        
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setFps(35)

        # Optimize StereoDepth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(camRgb.getPreviewWidth(), camRgb.getPreviewHeight())
        stereo.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5)  # Balance between accuracy and performance
        
        # Enable stereo depth optimization features
        stereo.setLeftRightCheck(True)  # Improves accuracy
        stereo.setExtendedDisparity(False)  # Disable if not needed for close objects
        stereo.setSubpixel(False)  # Disable for better performance
        
        # Configure confidence threshold to filter out low confidence depth data
        stereo.initialConfig.setConfidenceThreshold(200)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

        # Network specific settings
        spatialDetectionNetwork.setBlobPath(self.nnPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.05)  # Increased to reduce false positives
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(8000)  # Reduced from 10000 to focus on relevant range
        spatialDetectionNetwork.setNumClasses(80)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setAnchors([])
        spatialDetectionNetwork.setAnchorMasks({})
        spatialDetectionNetwork.setIouThreshold(0.5)

        # Enable hardware neural inference acceleration
        spatialDetectionNetwork.setNumInferenceThreads(2)
        spatialDetectionNetwork.input.setQueueSize(1)  # Minimize latency by processing latest frame
        
        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        spatialDetectionNetwork.out.link(nnOut.input)
        stereo.depth.link(xoutDepth.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)

        # Configure queue behavior for outputs
        for out in [xoutRgb, nnOut, xoutDepth]:
            out.input.setQueueSize(1)  # Keep only latest frame
            out.input.setBlocking(False)  # Non-blocking mode

        self.pipeline = pipeline

    def calculate_angle(self, frame_width, x_pos):
        frame_center_x = frame_width/2
        return ((x_pos - frame_center_x) / frame_center_x) 

    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ball_moved = False
        self.throttle_start_time = None   # Comment this line out if you want it to stop after 2s of driving, otherwise it stops after 2s of 						not seeing a ball
    
    def safety_check(self):
        # Stop the robot if no commands received recently
        if time.time() - self.last_throttle_command > self.safety_timeout:
            self.stop_robot()
    
    def publish_values(self):
        if self.emergency_stop:
            self.stop_robot()
            return
            
        # Create Twist message
        cmd_vel_msg = Twist()
        
        if self.ball_moved:
            # Check if we need to start the throttle timer
            if self.throttle_start_time is None:
                self.throttle_start_time = time.time()
            
            # Check if we've exceeded the maximum throttle duration
            if time.time() - self.throttle_start_time > self.MAX_THROTTLE_DURATION:
                self.get_logger().warn('Maximum throttle duration exceeded - stopping')
                self.stop_robot()
                return
            
            # Normal throttle calculation
            throttle = 10.0
            Kp = 2
            cmd_vel_msg.linear.x = throttle
            angle = self.last_angle * Kp
            cmd_vel_msg.angular.z = float(max(-1, min(angle, 1)))
            self.last_throttle_command = time.time()
        else:
            self.stop_robot()


        # Publish the Twist message
        self.cmd_vel_pub.publish(cmd_vel_msg)

        # Print to terminal
        if self.ball_detected:
            self.get_logger().info(f'Ball detected - Depth: {self.last_depth:.1f}mm, Angle: {self.last_angle:.1f}, Throttle: {cmd_vel_msg.linear.x:.2f}')
        else:
            self.get_logger().warn(f'Ball not detected - Last known - Depth: {self.last_depth:.1f}mm, Angle: {self.last_angle:.1f}')
        

        # Create Twist message
        """
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0 # float(self.last_depth)  # depth in mm
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = float(self.last_angle)  # angle in degrees
        
        # Publish twist message
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Print to terminal
        if self.ball_detected:
            self.get_logger().info(f'Ball found - Depth: {self.last_depth:.1f}mm, Angle: {self.last_angle:.1f}°')
        else:
            self.get_logger().warn(f'Ball not found - Last known - Depth: {self.last_depth:.1f}mm, Angle: {self.last_angle:.1f}°')
        """

    def run(self):
        with dai.Device(self.pipeline) as device:
            # Set up high-performance mode
            device.setLogLevel(dai.LogLevel.WARN)
            device.setLogOutputLevel(dai.LogLevel.WARN)
            
            # Output queues with optimized settings
            qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
            qDet = device.getOutputQueue(name="nn", maxSize=1, blocking=False)
            qDepth = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
            
            while rclpy.ok():
                inRgb = qRgb.get()
                inDet = qDet.get()

                if inRgb is not None:
                    frame = inRgb.getCvFrame()

                ball_found_this_frame = False
                if inDet is not None:
                    detections = inDet.detections

                    for detection in detections:
                        if detection.label == 32:  # Sports ball class
                            ball_found_this_frame = True
                            bbox = self.frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                            
                            # Calculate center point
                            center_x = (bbox[0] + bbox[2]) // 2
                            current_depth = detection.spatialCoordinates.z

			    # Initialize depth threshold if not set
                            if self.initial_depth is None:
                                self.initial_depth = current_depth
                            else:
                                # Update initial depth only if current depth is greater
                                self.initial_depth = max(self.initial_depth, current_depth)

                            # Check if the ball has been kicked towards the robot
                            if not self.ball_moved:
                                if current_depth < (self.initial_depth - self.movement_tolerance):
                                    self.consecutive_closer_frames += 1
                                    if self.consecutive_closer_frames >= self.frames_required:
                                        self.ball_moved = True
                                else:
                                    self.consecutive_closer_frames = 0

                            # Update values
                            self.last_angle = self.calculate_angle(frame.shape[1], center_x)
                            self.last_depth = detection.spatialCoordinates.z
                            self.ball_detected = True
                            
                            if frame is not None:
                                # Draw bounding box
                                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                                
                                # Display information
                                cv2.putText(frame, f"D:{int(self.last_depth)}mm A:{self.last_angle:.1f}°",
                                           (bbox[0] + 10, bbox[1] + 20),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if not ball_found_this_frame:
                    self.ball_detected = False

                # Publish values regardless of whether ball was detected
                self.publish_values()

                if frame is not None:
                    cv2.imshow("Ball Tracking", frame)

                if cv2.waitKey(1) == ord('q'):
                    break

            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    ball_tracker = BallTracker()
    
    try:
        ball_tracker.run()
    except KeyboardInterrupt:
        pass
    finally:
        ball_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
