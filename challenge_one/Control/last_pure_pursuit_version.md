# Last Pure Pursuit Version - Complete Source Code

This document contains all Pure Pursuit controller files and source code as of the last stable version.

**Excluded files:**
- `comprehensive_path_gen.py`
- `validation_node.py`
- `validator_complete.py`

---

## 1. AdaptiveLd_fixed.py

**Main Adaptive Pure Pursuit Controller**

```python
#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Adaptive Pure Pursuit controller — corrected version.

Changes from submitted code:
  FIX 1  self.distLd initialised to self.KC (= 0.1) instead of 0.5.
          0.5 exceeds the maximum valid lookahead (KL·Vmax+KC = 0.4 m)
          and causes an overshooting first step.
  FIX 2  In-place turn branch restored for |theta| > π/2.
          Without it, when the goal is behind the robot theta ≈ ±π,
          sin(π) = 0 → k = 0 → ω = 0 and the robot drives straight
          away from the goal (Test 4 fails).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import numpy as np


def euler_from_quaternion(quat):
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Control(Node):
    """Adaptive pure pursuit controller"""

    def __init__(self):
        super().__init__("adaptive_controller_fixed")

        self.pose_topic = self.declare_parameter(
            "subscribing_topics/pose", "/odometry/filtered"
        ).value
        self.path_topic = self.declare_parameter(
            "subscribing_topics/path", "/path"
        ).value
        
        self.simulate_pose = self.declare_parameter("simulate_pose", True).value
        self.MAXVELOCITY = self.declare_parameter("robot_parameters/maxVelocity", 1.0).value
        self.WIDTH       = self.declare_parameter("robot_parameters/width",       0.973).value
        # Wheel diameter is 0.25 m → radius 0.125 m
        self.wheel_radius = self.declare_parameter("robot_parameters/wheel_radius", 0.125).value
        self.KL          = self.declare_parameter("algorithm_parameters/KL",      0.3).value
        self.KC          = self.declare_parameter("algorithm_parameters/KC",      0.1).value
        self.min_speed   = self.declare_parameter("robot_parameters/min_speed",   0.10).value
        self.min_lookahead = self.declare_parameter("robot_parameters/min_lookahead", 0.2).value
        self.max_lookahead = self.declare_parameter("robot_parameters/max_lookahead", 1.2).value
        self.localPath   = self.declare_parameter("algorithm_parameters/localPath", False).value
        self.log         = self.declare_parameter("meta_parameters/log",           False).value
        self.min_turn_radius      = self.declare_parameter("robot_parameters/min_turn_radius",      1.0).value
        self.min_turn_speed_ratio = self.declare_parameter("robot_parameters/min_turn_speed_ratio",0.4).value
        self.publish_wheel_velocities = self.declare_parameter("robot_parameters/publish_wheel_velocities", True).value
        self.cmd_vel_topic = self.declare_parameter("publishing_topics/cmd_vel", "/cmd_vel").value
        self.wheel_topic = self.declare_parameter("publishing_topics/wheel_velocities", "/wheel_velocities").value

        # Prevent publishing two incompatible types on the same topic
        if self.publish_wheel_velocities and self.wheel_topic == self.cmd_vel_topic:
            self.get_logger().warn(
                "wheel_velocities topic must not be the same as cmd_vel because Twist and Float32MultiArray are incompatible. "
                "Disabling wheel velocity publication."
            )
            self.publish_wheel_velocities = False

        # Standard QoS for high-frequency control data
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.cmd_pub  = self.create_publisher(Twist,  self.cmd_vel_topic, self.control_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", self.control_qos)
        self.test_pub = self.create_publisher(String, "/control_test", 10)
        if self.publish_wheel_velocities:
            self.wheel_pub = self.create_publisher(Float32MultiArray, self.wheel_topic, self.control_qos)

        if not self.simulate_pose:
            self.create_subscription(Odometry, self.pose_topic, self.updatePose,    self.control_qos)
        
        self.create_subscription(Path,     self.path_topic, self.pathCallback,  self.control_qos)

        self.indexLD         = 0
        self.currentPosition = [0.0, -2.0, math.pi / 2.0]   # x, y, yaw (facing +Y)
        self.pose_received   = False

        if self.simulate_pose:
            self.pose_received = True

        # FIX 1: initialise to KC, not 0.5
        # Maximum valid value is KL*Vmax+KC = 0.3*1.0+0.1 = 0.4 m.
        # Starting at 0.5 overshoots that ceiling on the very first tick.
        self.distLd = self.KC

        self.timer_period = 0.02
        self.timer        = self.create_timer(self.timer_period, self.purePursuit)
        self.waypoints    = []

    def pathCallback(self, msg: Path):
        self.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        if self.log:
            self.get_logger().info(f"got waypoints: {len(self.waypoints)}")

    def updatePose(self, msg: Odometry):
        if not self.pose_received:
            self.get_logger().info("First odom received!")
        self.currentPosition[0] = msg.pose.pose.position.x
        self.currentPosition[1] = msg.pose.pose.position.y
        orientation     = msg.pose.pose.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw       = euler_from_quaternion(orientationList)
        self.currentPosition[2] = yaw
        self.pose_received = True

    def setVelocity(self, k: float) -> float:
        velocity = self.MAXVELOCITY / (1.0 + abs(k))
        velocity = max(self.min_speed, min(self.MAXVELOCITY, velocity))
        goalPoint = self.waypoints[-1]
        distanceToGoal = np.linalg.norm(
            np.array((self.currentPosition[0], self.currentPosition[1]))
            - np.array(goalPoint)
        )
        if distanceToGoal < 0.2:
            return 0.0
        return float(velocity)

    def findLookaheadPoint(self, robotPosition: tuple):
        if not self.waypoints:
            return None

        robot_pt = np.array(robotPosition)
        dists = np.linalg.norm(np.array(self.waypoints) - robot_pt, axis=1)
        closest_idx = int(np.argmin(dists))

        for i in range(closest_idx, len(self.waypoints)):
            waypoint = self.waypoints[i]
            distanceToRobot = np.linalg.norm(np.array(waypoint) - robot_pt)
            if distanceToRobot >= self.distLd:
                self.indexLD = i
                return waypoint

        return self.waypoints[-1]

    def purePursuit(self):
        if not self.pose_received:
            self.get_logger().info(
                "Waiting for odom on '%s'..." % self.pose_topic,
                throttle_duration_sec=3.0,
            )
            return
        if len(self.waypoints) == 0:
            self.get_logger().info(
                "Waiting for waypoints on '%s'..." % self.path_topic,
                throttle_duration_sec=3.0,
            )
            return

        lookaheadPoint = self.findLookaheadPoint(
            (self.currentPosition[0], self.currentPosition[1])
        )
        if lookaheadPoint is None:
            self.get_logger().warn("Lookahead point is None!")
            return

        dx = lookaheadPoint[0] - self.currentPosition[0]
        dy = lookaheadPoint[1] - self.currentPosition[1]
        Ld = math.hypot(dx, dy) + 1e-6

        headingX = math.cos(self.currentPosition[2])
        headingY = math.sin(self.currentPosition[2])

        dot   = dx * headingX + dy * headingY
        det   = headingX * dy - headingY * dx
        theta = math.atan2(det, dot)
        
        # Use distLd (desired) instead of Ld (actual) to avoid 1/0 and for stability
        # Pure Pursuit: k = 2*sin(theta)/Ld
        k = 2.0 * math.sin(theta) / max(0.1, self.distLd)
        
        # Cap curvature to avoid extreme angular velocities
        k = max(-5.0, min(5.0, k))

        v = self.setVelocity(k)

        # In-place turn branch: recover when the lookahead goal lies behind the robot.
        # Pure pursuit curvature goes to zero near |theta| ≈ π, which can drive straight away
        # from the path instead of turning back toward the goal.
        if abs(theta) > (math.pi / 2):
            v = min(v, 0.4 * self.MAXVELOCITY)
            max_inplace_ang = 1.0
            angular = math.copysign(max_inplace_ang, theta)
            mode = "IN-PLACE"
        else:
            angular = v * k
            angular = max(min(angular, 1.5), -1.5)
            mode = "NORMAL"

        if self.log:
            self.get_logger().info(
                f"Ld_pt: ({lookaheadPoint[0]:.2f}, {lookaheadPoint[1]:.2f}), "
                f"Ld_dist: {math.hypot(dx, dy):.2f}, "
                f"theta: {math.degrees(theta):.1f}°, k: {k:.2f}, "
                f"v: {v:.2f}, w: {angular:.2f}, mode: {mode}",
                throttle_duration_sec=0.5
            )

        # Update distLd for next tick based on current velocity
        self.distLd = self.KL * v + self.KC
        self.distLd = max(self.min_lookahead, min(self.max_lookahead, self.distLd))

        tw = Twist()
        tw.linear.x   = float(v)
        tw.angular.z  = float(angular)
        self.cmd_pub.publish(tw)

        if self.publish_wheel_velocities:
            # Publish six wheel angular velocities (rad/s) as Float32MultiArray
            # Differential-drive kinematics:
            # wL = (v - omega * WIDTH/2) / r
            # wR = (v + omega * WIDTH/2) / r
            r = float(self.wheel_radius)
            w_left = (v - angular * (self.WIDTH / 2.0)) / max(1e-6, r)
            w_right = (v + angular * (self.WIDTH / 2.0)) / max(1e-6, r)

            wheel_msg = Float32MultiArray()
            wheel_msg.data = [float(w_left), float(w_left), float(w_left), float(w_right), float(w_right), float(w_right)]
            self.wheel_pub.publish(wheel_msg)

        if self.log:
            self.get_logger().info(f"Published cmd_vel: v={v:.2f}, w={angular:.2f}", throttle_duration_sec=1.0)

        test_msg      = String()
        yaw_deg       = math.degrees(self.currentPosition[2])
        direction     = "straight"
        if angular >  0.01: direction = "left"
        elif angular < -0.01: direction = "right"
        test_msg.data = (
            f"yaw: {yaw_deg:.1f}°, velocity: {v:.3f} m/s, "
            f"angular: {angular:.3f} rad/s, direction: {direction}"
        )
        self.test_pub.publish(test_msg)

        if self.simulate_pose:
            dt  = self.timer_period
            yaw = self.currentPosition[2]
            self.currentPosition[0] += v * math.cos(yaw) * dt
            self.currentPosition[1] += v * math.sin(yaw) * dt
            self.currentPosition[2] += angular * dt
            # Wrap yaw to [-pi, pi]
            self.currentPosition[2] = (self.currentPosition[2] + math.pi) % (2 * math.pi) - math.pi
            
            # Publish simulated odom
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = self.currentPosition[0]
            odom.pose.pose.position.y = self.currentPosition[1]
            # yaw to quaternion
            odom.pose.pose.orientation.z = math.sin(self.currentPosition[2] / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.currentPosition[2] / 2.0)
            self.odom_pub.publish(odom)

            if self.log:
                self.get_logger().info(
                    f"Sim pose: x={self.currentPosition[0]:.3f}, "
                    f"y={self.currentPosition[1]:.3f}, "
                    f"yaw={math.degrees(self.currentPosition[2]):.1f}°",
                    throttle_duration_sec=0.5,
                )


def main(args=None):
    rclpy.init(args=args)
    control_node = Control()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 2. __init__.py

**Package Init File**

```python
# Empty package initialization file
```

---

## 3. pathTest.py

**Path Publisher for Testing**

```python
#!/usr/bin/env python3

# pylint: disable=all
# mypy: ignore-errors

import math
import rclpy
from rclpy.node import Node
#from turtlebot3_msgs.msg import wp_list  # Import your custom message
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class path_publisher(Node):
    def __init__(self):
        super().__init__('path_test_publisher')
        
        # Defining parameters
        self.DS = 0.05
        self.R_turn = 0.5  # Sharp turn radius
        self.R_circle = 1.5 # Main circle radius

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Path, '/path', self.qos)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.msg = Path()
        self.msg.header.frame_id = "odom"
        self.generate_track_message()
        
    def generate_track_message(self):
        # Create a path with: Straight -> U-turn -> Circle
        x_segments = []
        y_segments = []

        def append_segment(xs, ys):
            x_segments.append(xs)
            y_segments.append(ys)

        # 1) Straight line forward (0,0) -> (4,0)
        x1 = np.linspace(0.0, 4.0, int(4.0 / self.DS) + 1)
        y1 = np.zeros_like(x1)
        append_segment(x1, y1)

        # 2) U-turn (180 degrees)
        # Semicircle center at (4.0, 1.0), radius 1.0
        # From -pi/2 to pi/2
        R_uturn = 1.0
        theta2 = np.linspace(-math.pi / 2, math.pi / 2, int(math.pi * R_uturn / self.DS) + 1)
        x2 = 4.0 + R_uturn * np.cos(theta2)
        y2 = 1.0 + R_uturn * np.sin(theta2)
        append_segment(x2, y2)

        # 3) Straight back part of U-turn (4,2) -> (2,2)
        x3 = np.linspace(4.0, 2.0, int(2.0 / self.DS) + 1)
        y3 = np.full_like(x3, 2.0)
        append_segment(x3, y3)

        # 4) Circular path
        # Circle center at (0.5, 2.0), radius 1.5
        # Starts at (2,2), goes full circle
        R_circle = 1.5
        cx, cy = 0.5, 2.0
        theta4 = np.linspace(0.0, 2 * math.pi, int(2 * math.pi * R_circle / self.DS) + 1)
        x4 = cx + R_circle * np.cos(theta4)
        y4 = cy + R_circle * np.sin(theta4)
        append_segment(x4, y4)

        self.x_way_points_list = np.concatenate(x_segments)
        self.y_way_points_list = np.concatenate(y_segments)

        # Save to CSV for APF to load
        import pandas as pd
        import os
        results_dir = os.path.expanduser("~/ROAR-Nouveau-Autonomous-System/Path-planning/heightmap_costmap/Results")
        os.makedirs(results_dir, exist_ok=True)
        df = pd.DataFrame({
            'real_x': self.x_way_points_list,
            'real_y': self.y_way_points_list
        })
        csv_path = os.path.join(results_dir, "complex_path.csv")
        df.to_csv(csv_path, index=False)
        self.get_logger().info(f"Saved modified path (U-turn + Circle) to {csv_path}")

        self.msg.poses = []
        for x, y in zip(self.x_way_points_list, self.y_way_points_list):
            point = PoseStamped()
            point.pose.position.x = float(x)
            point.pose.position.y = float(y)
            self.msg.poses.append(point)

        self.get_logger().info(f"Generated U-turn path with {len(self.x_way_points_list)} waypoints.")    

    def publish_path(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.msg)
        self.get_logger().info("Published path message")

def main(args=None):
    rclpy.init(args=args)
    
    node = path_publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 4. performance_monitor.py

**Performance Monitoring Module**

```python
#!/usr/bin/env python3
"""
Generic Performance Monitor Module

A flexible performance monitoring system that can track timing, metrics, and statistics
for any application. Provides both real-time monitoring and logging capabilities.

Usage:
    monitor = PerformanceMonitor(enable_logging=True, log_file_path="performance.log")
    monitor.start_frame()
    
    monitor.start_timer("processing_stage")
    # ... do some work ...
    duration = monitor.end_timer("processing_stage")
    
    monitor.record_metric("items_processed", 100)
    monitor.end_frame()
    monitor.log_metrics()
"""

import time
import json
import csv
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from collections import defaultdict, deque
import threading


@dataclass
class PerformanceMetrics:
    """Container for performance metrics data"""
    # Timing data
    timers: Dict[str, float] = field(default_factory=dict)
    
    # Custom metrics
    metrics: Dict[str, Any] = field(default_factory=dict)
    
    # Frame information
    frame_id: str = ""
    timestamp: float = 0.0
    
    # Processing statistics
    total_time_ms: float = 0.0
    frame_count: int = 0
    
    # Performance flags
    flags: Dict[str, bool] = field(default_factory=dict)


@dataclass
class MonitorParams:
    """Configuration parameters for the performance monitor"""
    enable_detailed_timing: bool = True
    enable_debug_output: bool = True
    enable_performance_logging: bool = True
    timing_report_interval: float = 1.0  # seconds
    log_file_path: str = ""
    max_history_size: int = 100
    csv_logging: bool = False
    json_logging: bool = False


class PerformanceMonitor:
    """
    Generic Performance Monitor for tracking timing and metrics
    
    Features:
    - Multi-stage timing measurement
    - Custom metrics recording
    - Real-time statistics
    - File logging (CSV/JSON)
    - Thread-safe operation
    """
    
    def __init__(self, params: Optional[MonitorParams] = None):
        """
        Initialize the performance monitor
        
        Args:
            params: Configuration parameters, uses defaults if None
        """
        self.params = params or MonitorParams()
        
        # Current frame data
        self.current_metrics = PerformanceMetrics()
        self.active_timers: Dict[str, float] = {}
        
        # Historical data for statistics
        self.frame_times: deque = deque(maxlen=self.params.max_history_size)
        self.timer_history: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=self.params.max_history_size)
        )
        self.metric_history: Dict[str, deque] = defaultdict(
            lambda: deque(maxlen=self.params.max_history_size)
        )
        
        # Statistics
        self.frame_count = 0
        self.last_report_time = time.time()
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Initialize logging
        if self.params.log_file_path and self.params.enable_performance_logging:
            self._initialize_logging()
    
    def _initialize_logging(self):
        """Initialize log files"""
        if self.params.csv_logging:
            self._create_csv_header()
        if self.params.json_logging:
            self._create_json_log()
    
    def _create_csv_header(self):
        """Create CSV file with headers"""
        try:
            with open(self.params.log_file_path + '.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Basic headers - will be extended dynamically
                headers = ['timestamp', 'frame_id', 'total_time_ms', 'frame_count']
                writer.writerow(headers)
        except Exception as e:
            print(f"Warning: Could not create CSV log file: {e}")
    
    def _create_json_log(self):
        """Create JSON log file"""
        try:
            with open(self.params.log_file_path + '.json', 'w') as jsonfile:
                json.dump({"performance_log": []}, jsonfile)
        except Exception as e:
            print(f"Warning: Could not create JSON log file: {e}")
    
    def start_frame(self, frame_id: str = ""):
        """
        Start timing a new frame/iteration
        
        Args:
            frame_id: Optional identifier for this frame
        """
        with self._lock:
            self.current_metrics = PerformanceMetrics()
            self.current_metrics.frame_id = frame_id
            self.current_metrics.timestamp = time.time()
            self.current_metrics.frame_count = self.frame_count
            self.active_timers.clear()
    
    def end_frame(self):
        """End the current frame and update statistics"""
        with self._lock:
            end_time = time.time()
            self.current_metrics.total_time_ms = (end_time - self.current_metrics.timestamp) * 1000
            
            # Update statistics
            self.frame_times.append(self.current_metrics.total_time_ms)
            self.frame_count += 1
            
            # Update timer history
            for timer_name, duration in self.current_metrics.timers.items():
                self.timer_history[timer_name].append(duration)
            
            # Update metric history
            for metric_name, value in self.current_metrics.metrics.items():
                if isinstance(value, (int, float)):
                    self.metric_history[metric_name].append(value)
    
    def start_timer(self, stage_name: str):
        """
        Start timing a specific stage
        
        Args:
            stage_name: Name of the stage to time
        """
        with self._lock:
            self.active_timers[stage_name] = time.time()
    
    def end_timer(self, stage_name: str) -> float:
        """
        End timing for a specific stage
        
        Args:
            stage_name: Name of the stage to end timing for
            
        Returns:
            Duration in milliseconds
        """
        with self._lock:
            if stage_name in self.active_timers:
                end_time = time.time()
                duration_ms = (end_time - self.active_timers[stage_name]) * 1000
                self.current_metrics.timers[stage_name] = duration_ms
                del self.active_timers[stage_name]
                return duration_ms
            return 0.0
    
    def record_metric(self, metric_name: str, value: Any):
        """
        Record a custom metric
        
        Args:
            metric_name: Name of the metric
            value: Value to record (can be any type)
        """
        with self._lock:
            self.current_metrics.metrics[metric_name] = value
    
    def set_flag(self, flag_name: str, value: bool = True):
        """
        Set a performance flag
        
        Args:
            flag_name: Name of the flag
            value: Flag value
        """
        with self._lock:
            self.current_metrics.flags[flag_name] = value
    
    def get_current_metrics(self) -> PerformanceMetrics:
        """Get current frame metrics"""
        with self._lock:
            # Create a deep copy to avoid race conditions
            metrics_copy = PerformanceMetrics()
            metrics_copy.frame_id = self.current_metrics.frame_id
            metrics_copy.timestamp = self.current_metrics.timestamp
            metrics_copy.total_time_ms = self.current_metrics.total_time_ms
            metrics_copy.frame_count = self.current_metrics.frame_count
            
            # Copy dictionaries to avoid modification during iteration
            metrics_copy.timers = dict(self.current_metrics.timers)
            metrics_copy.metrics = dict(self.current_metrics.metrics)
            metrics_copy.flags = dict(self.current_metrics.flags)
            
            return metrics_copy
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get performance statistics
        
        Returns:
            Dictionary containing various statistics
        """
        with self._lock:
            stats = {
                'frame_count': self.frame_count,
                'avg_frame_time_ms': 0.0,
                'min_frame_time_ms': 0.0,
                'max_frame_time_ms': 0.0,
                'timer_stats': {},
                'metric_stats': {}
            }
            
            if self.frame_times:
                stats['avg_frame_time_ms'] = sum(self.frame_times) / len(self.frame_times)
                stats['min_frame_time_ms'] = min(self.frame_times)
                stats['max_frame_time_ms'] = max(self.frame_times)
            
            # Timer statistics
            for timer_name, history in self.timer_history.items():
                if history:
                    stats['timer_stats'][timer_name] = {
                        'avg_ms': sum(history) / len(history),
                        'min_ms': min(history),
                        'max_ms': max(history),
                        'count': len(history)
                    }
            
            # Metric statistics
            for metric_name, history in self.metric_history.items():
                if history:
                    stats['metric_stats'][metric_name] = {
                        'avg': sum(history) / len(history),
                        'min': min(history),
                        'max': max(history),
                        'count': len(history)
                    }
            
            return stats
    
    def print_summary(self):
        """Print a summary of current performance"""
        current = self.get_current_metrics()
        stats = self.get_statistics()
        
        print(f"\n=== PERFORMANCE SUMMARY ===")
        print(f"Frame: {current.frame_id} | Total time: {current.total_time_ms:.2f} ms")
        print(f"Frame count: {self.frame_count}")
        
        if current.timers:
            print(f"\nTiming breakdown:")
            # Create a copy to avoid modification during iteration
            timers_copy = dict(current.timers)
            for stage, duration in timers_copy.items():
                print(f"  {stage}: {duration:.2f} ms")
        
        if current.metrics:
            print(f"\nMetrics:")
            # Create a copy to avoid modification during iteration
            metrics_copy = dict(current.metrics)
            for name, value in metrics_copy.items():
                print(f"  {name}: {value}")
        
        if stats['avg_frame_time_ms'] > 0:
            print(f"\nStatistics (last {len(self.frame_times)} frames):")
            print(f"  Avg frame time: {stats['avg_frame_time_ms']:.2f} ms")
            print(f"  Min frame time: {stats['min_frame_time_ms']:.2f} ms")
            print(f"  Max frame time: {stats['max_frame_time_ms']:.2f} ms")
    
    def print_detailed_timing(self):
        """Print detailed timing information"""
        current = self.get_current_metrics()
        
        print(f"\n=== DETAILED TIMING BREAKDOWN ===")
        print(f"Frame ID: {current.frame_id}")
        print(f"Timestamp: {datetime.fromtimestamp(current.timestamp)}")
        print(f"Total time: {current.total_time_ms:.2f} ms")
        
        if current.timers:
            print(f"\nStage timings:")
            # Create a copy to avoid modification during iteration
            timers_copy = dict(current.timers)
            for stage, duration in sorted(timers_copy.items()):
                percentage = (duration / current.total_time_ms * 100) if current.total_time_ms > 0 else 0
                print(f"  {stage:20s}: {duration:8.2f} ms ({percentage:5.1f}%)")
    
    def log_metrics(self):
        """Log current metrics to file"""
        if not self.params.enable_performance_logging or not self.params.log_file_path:
            return
        
        current = self.get_current_metrics()
        
        try:
            if self.params.csv_logging:
                self._log_to_csv(current)
            if self.params.json_logging:
                self._log_to_json(current)
        except Exception as e:
            print(f"Warning: Failed to log metrics: {e}")
    
    def _log_to_csv(self, metrics: PerformanceMetrics):
        """Log metrics to CSV file"""
        csv_path = self.params.log_file_path + '.csv'
        
        # Prepare row data
        row = [
            metrics.timestamp,
            metrics.frame_id,
            metrics.total_time_ms,
            metrics.frame_count
        ]
        
        # Add timer data
        for timer_name, duration in metrics.timers.items():
            row.append(duration)
        
        # Add metric data
        for metric_name, value in metrics.metrics.items():
            row.append(value)
        
        with open(csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)
    
    def _log_to_json(self, metrics: PerformanceMetrics):
        """Log metrics to JSON file"""
        json_path = self.params.log_file_path + '.json'
        
        # Convert metrics to dictionary
        log_entry = {
            'timestamp': metrics.timestamp,
            'frame_id': metrics.frame_id,
            'total_time_ms': metrics.total_time_ms,
            'frame_count': metrics.frame_count,
            'timers': metrics.timers,
            'metrics': metrics.metrics,
            'flags': metrics.flags
        }
        
        # Read existing data
        try:
            with open(json_path, 'r') as jsonfile:
                data = json.load(jsonfile)
        except (FileNotFoundError, json.JSONDecodeError):
            data = {"performance_log": []}
        
        # Add new entry
        data["performance_log"].append(log_entry)
        
        # Write back
        with open(json_path, 'w') as jsonfile:
            json.dump(data, jsonfile, indent=2)
    
    def reset(self):
        """Reset all statistics and history"""
        with self._lock:
            self.current_metrics = PerformanceMetrics()
            self.active_timers.clear()
            self.frame_times.clear()
            self.timer_history.clear()
            self.metric_history.clear()
            self.frame_count = 0
            self.last_report_time = time.time()
    
    def set_params(self, params: MonitorParams):
        """Update monitor parameters"""
        with self._lock:
            self.params = params
            if self.params.log_file_path and self.params.enable_performance_logging:
                self._initialize_logging()


# Convenience functions for quick usage
def create_monitor(enable_logging: bool = True, 
                  log_file_path: str = "",
                  csv_logging: bool = False,
                  json_logging: bool = False) -> PerformanceMonitor:
    """
    Create a performance monitor with common settings
    
    Args:
        enable_logging: Enable file logging
        log_file_path: Path for log files
        csv_logging: Enable CSV logging
        json_logging: Enable JSON logging
        
    Returns:
        Configured PerformanceMonitor instance
    """
    params = MonitorParams(
        enable_performance_logging=enable_logging,
        log_file_path=log_file_path,
        csv_logging=csv_logging,
        json_logging=json_logging
    )
    return PerformanceMonitor(params)


# Context manager for easy timing
class TimerContext:
    """Context manager for timing code blocks"""
    
    def __init__(self, monitor: PerformanceMonitor, stage_name: str):
        self.monitor = monitor
        self.stage_name = stage_name
    
    def __enter__(self):
        self.monitor.start_timer(self.stage_name)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.monitor.end_timer(self.stage_name)


# Decorator for timing functions
def time_function(monitor: PerformanceMonitor, stage_name: str = None):
    """
    Decorator to automatically time function execution
    
    Args:
        monitor: PerformanceMonitor instance
        stage_name: Name for the timer (defaults to function name)
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            timer_name = stage_name or func.__name__
            monitor.start_timer(timer_name)
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                monitor.end_timer(timer_name)
        return wrapper
    return decorator
```

---

## 5. visual_control.py

**Adaptive Pure Pursuit Controller with RViz Visualization**

```python
#!/usr/bin/env python3

# pylint: disable=all
# mypy: ignore-errors

"""Adaptive Pure Pursuit controller with RViz visualization (consumes /path and /odometry/filtered, publishes /cmd_vel and /visualization_marker)"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import numpy as np


def euler_from_quaternion(quat):
    """Convert quaternion [x, y, z, w] to euler [roll, pitch, yaw]."""
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class VisualControl(Node):
    """Adaptive pure pursuit controller with RViz visualization"""

    def __init__(self):
        super().__init__("visual_controller")

        self.pose_topic = self.declare_parameter(
            "subscribing_topics/pose", "/odometry/filtered"
        ).value
        self.path_topic = self.declare_parameter(
            "subscribing_topics/path", "/path"
        ).value

        # Standard QoS for high-frequency control data
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", self.control_qos)
        self.test_pub = self.create_publisher(String, "/control_test", 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

        self.create_subscription(Odometry, self.pose_topic, self.updatePose, self.control_qos)
        self.create_subscription(Path, self.path_topic, self.pathCallback, self.control_qos)

        self.indexLD = 0
        self.currentPosition = [0.0, 0.0, 0.0]
        self.pose_received = False

        self.simulate_pose = self.declare_parameter("simulate_pose", True).value
        if self.simulate_pose:
            self.pose_received = True

        self.MAXVELOCITY = self.declare_parameter("robot_parameters/maxVelocity", 1.0).value
        self.WIDTH = self.declare_parameter("robot_parameters/width", 0.5).value
        self.KL = self.declare_parameter("algorithm_parameters/KL", 0.6).value
        self.KC = self.declare_parameter("algorithm_parameters/KC", 0.4).value
        self.min_speed = self.declare_parameter("robot_parameters/min_speed", 0.10).value
        self.min_lookahead = self.declare_parameter("robot_parameters/min_lookahead", 0.2).value
        self.max_lookahead = self.declare_parameter("robot_parameters/max_lookahead", 1.2).value
        self.localPath = self.declare_parameter("algorithm_parameters/localPath", False).value
        self.log = self.declare_parameter("meta_parameters/log", False).value

        # FIX 1: initialise to KC, not 0.5
        # Maximum valid value is KL*Vmax+KC = 0.6*1.0+0.4 = 1.0 m.
        # Starting at 0.5 overshoots that ceiling on the very first tick.
        self.distLd = self.KC

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.purePursuit)

        self.waypoints = []

    def pathCallback(self, msg: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.indexLD = 0
        if self.log:
            self.get_logger().info(f"got waypoints: {len(self.waypoints)}")

    def updatePose(self, msg: Odometry):
        self.currentPosition[0] = msg.pose.pose.position.x
        self.currentPosition[1] = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientationList)
        self.currentPosition[2] = yaw
        self.pose_received = True

    def setVelocity(self, k: float) -> float:
        velocity = self.MAXVELOCITY / (1.0 + abs(k))
        velocity = max(self.min_speed, min(self.MAXVELOCITY, velocity))
        goalPoint = self.waypoints[-1]
        distanceToGoal = np.linalg.norm(
            np.array((self.currentPosition[0], self.currentPosition[1])) - np.array(goalPoint)
        )
        if distanceToGoal < 0.2:
            return 0.0
        return float(velocity)

    def findLookaheadPoint(self, robotPosition: tuple):
        for i in range(self.indexLD, len(self.waypoints)):
            waypoint = self.waypoints[i]
            distanceToRobot = np.linalg.norm(
                np.array(waypoint) - np.array(robotPosition)
            )
            if distanceToRobot >= self.distLd:
                self.indexLD = i
                return waypoint
        return self.waypoints[-1]

    def purePursuit(self):
        if not self.pose_received:
            self.get_logger().info(
                "Waiting for odom on '%s'..." % self.pose_topic,
                throttle_duration_sec=3.0,
            )
            return

        if len(self.waypoints) == 0:
            return

        lookaheadPoint = self.findLookaheadPoint((self.currentPosition[0], self.currentPosition[1]))
        if lookaheadPoint is None:
            return

        dx = lookaheadPoint[0] - self.currentPosition[0]
        dy = lookaheadPoint[1] - self.currentPosition[1]
        Ld = math.hypot(dx, dy) + 1e-6

        headingX = math.cos(self.currentPosition[2])
        headingY = math.sin(self.currentPosition[2])

        dot = dx * headingX + dy * headingY
        det = headingX * dy - headingY * dx
        theta = math.atan2(det, dot)
        k = 2.0 * math.sin(theta) / Ld

        v = self.setVelocity(k)
        self.distLd = self.KL * v + self.KC
        self.distLd = max(self.min_lookahead, min(self.max_lookahead, self.distLd))

        # FIX 2: restore in-place turn branch.
        # When |theta| > π/2 the goal is behind the robot.
        # Pure pursuit gives sin(theta) ≈ 0 → k ≈ 0 → ω = 0 (no turn).
        # The in-place branch overrides this: reduce speed and rotate toward the goal.
        if abs(theta) > (math.pi / 2):
            v = min(v, 0.4 * self.MAXVELOCITY)
            max_inplace_ang = 1.0   # rad/s gentle cap
            desired_omega = abs(v) / max(1e-3, 1.0)
            angular = math.copysign(min(max_inplace_ang, desired_omega), theta)
        else:
            angular = v * k
            max_ang = 2.0
            angular = max(min(angular, max_ang), -max_ang)

        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(angular)
        self.cmd_pub.publish(tw)

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "adaptive_pursuit"
        delete_marker.action = Marker.DELETEALL
        self.marker_pub.publish(delete_marker)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.ns = "adaptive_pursuit"
        lookahead_marker.id = 1
        lookahead_marker.type = Marker.SPHERE
        lookahead_marker.action = Marker.ADD
        lookahead_marker.pose.position.x = lookaheadPoint[0]
        lookahead_marker.pose.position.y = lookaheadPoint[1]
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        lookahead_marker.scale.x = 0.2
        lookahead_marker.scale.y = 0.2
        lookahead_marker.scale.z = 0.2
        lookahead_marker.color.a = 1.0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        self.marker_pub.publish(lookahead_marker)

        velocity_marker = Marker()
        velocity_marker.header.frame_id = "map"
        velocity_marker.header.stamp = self.get_clock().now().to_msg()
        velocity_marker.ns = "adaptive_pursuit"
        velocity_marker.id = 2
        velocity_marker.type = Marker.ARROW
        velocity_marker.action = Marker.ADD
        velocity_marker.pose.position.x = self.currentPosition[0]
        velocity_marker.pose.position.y = self.currentPosition[1]
        velocity_marker.pose.position.z = 0.0
        velocity_marker.pose.orientation.z = math.sin(self.currentPosition[2] / 2)
        velocity_marker.pose.orientation.w = math.cos(self.currentPosition[2] / 2)
        velocity_marker.scale.x = max(v * 2.0, 0.1)
        velocity_marker.scale.y = 0.05
        velocity_marker.scale.z = 0.05
        velocity_marker.color.a = 1.0
        velocity_marker.color.r = 0.0
        velocity_marker.color.g = 1.0
        velocity_marker.color.b = 0.0
        self.marker_pub.publish(velocity_marker)

        if self.log:
            self.get_logger().info(
                f"Ld_pt: ({lookaheadPoint[0]:.2f}, {lookaheadPoint[1]:.2f}), "
                f"Ld_dist: {math.hypot(dx, dy):.2f}, "
                f"theta: {math.degrees(theta):.1f}°, k: {k:.2f}, "
                f"v: {v:.2f}, w: {angular:.2f}",
                throttle_duration_sec=0.5
            )

        test_msg = String()
        yaw_deg = math.degrees(self.currentPosition[2])
        direction = "straight"
        if angular > 0.01:
            direction = "left"
        elif angular < -0.01:
            direction = "right"
        test_msg.data = f"yaw: {yaw_deg:.1f}°, velocity: {v:.3f} m/s, angular: {angular:.3f} rad/s, direction: {direction}"
        self.test_pub.publish(test_msg)

        if self.simulate_pose:
            dt = self.timer_period
            yaw = self.currentPosition[2]
            self.currentPosition[0] += v * math.cos(yaw) * dt
            self.currentPosition[1] += v * math.sin(yaw) * dt
            self.currentPosition[2] += angular * dt
            if self.log:
                self.get_logger().info(
                    f"Sim pose: x={self.currentPosition[0]:.3f}, "
                    f"y={self.currentPosition[1]:.3f}, "
                    f"yaw={math.degrees(self.currentPosition[2]):.1f}°",
                    throttle_duration_sec=0.5,
                )


def main(args=None):
    rclpy.init(args=args)
    node = VisualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
```

---

## End of Pure Pursuit Source Code

**File Location:** `/home/nouran/ROAR-Nouveau-Autonomous-System/Control/last_pure_pursuit_version.md`

This document serves as a complete backup and reference for all Pure Pursuit controller implementations.
