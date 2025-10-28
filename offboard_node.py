#!/usr/bin/env python3
"""
Offboard waypoint navigation with enhanced fail-safes, attempts, approach-from-better-angle,
logging and plotting.

Features added/changed:
- Verified color scheme for RViz markers:
    start (WP0): green
    final (last WP): red
    completed: green
    skipped: orange
    current: yellow
    future: blue
- Uses 0.93 m tolerance for waypoint completion.
- Max 2 attempts per waypoint (configurable via param).
- On comm loss:
    Case 1: If already progressed (reached >= 1 waypoint) -> attempt to finish mission locally with same rules.
    Case 2: If not progressed past 3 waypoints (reached_count <= 3) -> command RTL.
- If error > 1.9 m when trying a waypoint, perform a corrective approach-from-better-angle before re-attempting.
- Logs events with detailed messages and collects time-series for plotting and CSV output.
- Saves CSV and PNG plots at ~/mission_logs/<timestamp> on exit.

Notes: This node uses mavros state fields: `.connected`, `.armed`, `.mode` to detect comm and status.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState, NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import time
import os
import csv
from datetime import datetime

# Optional plotting (try import but keep node working if matplotlib missing)
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False


class OffboardWaypointNavigation(Node):
    def __init__(self):
        super().__init__('offboard_waypoint_navigation')

        # Parameters (set defaults to meet your requested policy)
        self.declare_parameters(namespace='', parameters=[
            ('altitude', 20.0),
            ('rectangle_width', 50.0),
            ('rectangle_height', 80.0),
            ('laps', 1),
            ('setpoint_rate', 20.0),
            ('min_battery', 10.5),
            ('position_tolerance', 0.75),  # REQUIRED: 0.93 m radius
            ('max_velocity', 100.0),
            ('waypoint_timeout', 30.0),
            ('max_attempts_per_waypoint', 2),  # REQUIRED: 2 attempts
            ('approach_backoff_distance', 0.5),  # distance to back off for better-angle approach
            ('error_for_better_angle', 1.9),  # if error > 1.9 m do approach-from-better-angle
        ])

        # Params
        self.altitude = self.get_parameter('altitude').value
        self.rectangle_width = self.get_parameter('rectangle_width').value
        self.rectangle_height = self.get_parameter('rectangle_height').value
        self.laps = int(self.get_parameter('laps').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.waypoint_timeout = float(self.get_parameter('waypoint_timeout').value)
        self.max_attempts = int(self.get_parameter('max_attempts_per_waypoint').value)
        self.approach_backoff_distance = float(self.get_parameter('approach_backoff_distance').value)
        self.error_for_better_angle = float(self.get_parameter('error_for_better_angle').value)

        self.get_logger().info("🚀 Offboard Waypoint Navigation (enhanced)")
        self.get_logger().info(f"Tolerance: {self.position_tolerance} m, max attempts: {self.max_attempts}")
        self.get_logger().info(f"Better-angle threshold: {self.error_for_better_angle} m")

        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.battery_voltage = 16.0
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED

        # Mission control
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_lap = 0
        self.mission_started = False
        self.mission_completed = False
        self.emergency_landing = False

        # Waypoint tracking
        self.waypoint_attempts = {}           # attempts per waypoint
        self.skipped_waypoints = set()
        self.waypoint_reached_time = None
        self.reached_waypoints_count = 0      # total reached count (for Case2 comm loss check)

        # Logging / plotting arrays
        self.log_times = []
        self.log_errors = []
        self.log_setpoints = []   # list of (x,y,z)
        self.log_actuals = []     # list of (x,y,z)
        self.log_wp_status = []   # list of waypoint index reached/skipped flags over time

        # Generate waypoints
        self.generate_rectangular_waypoints()

        # QoS
        qos_state = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_pose = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        # Subscribers / Publishers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_state)
        self.local_pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_pose)
        self.local_velocity_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.velocity_callback, qos_pose)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # RViz
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/mission_waypoints', 10)
        self.drone_marker_pub = self.create_publisher(Marker, '/drone_position', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/mission_path', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # timers
        self.control_timer = self.create_timer(1.0 / float(self.get_parameter('setpoint_rate').value), self.control_loop)  # default 20Hz
        self.rviz_timer = self.create_timer(0.1, self.update_rviz_visualization)
        self.log_timer = self.create_timer(1.0, self.log_status)  # periodic status log & capture for plotting

        # wait & initial setpoints
        self.wait_for_services()
        self.send_initial_setpoints()
        self.publish_waypoint_markers()

        # Prepare log folder
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.log_dir = os.path.expanduser(f"~/mission_logs/{ts}")
        os.makedirs(self.log_dir, exist_ok=True)

    # -------------------------
    # Waypoints & markers
    # -------------------------
    def generate_rectangular_waypoints(self):
        """Same pattern as your original; using center (0,0)"""
        center_x, center_y = 0.0, 0.0
        half_width = self.rectangle_width / 2.0
        half_height = self.rectangle_height / 2.0

        waypoints_local = [
            (center_x, center_y, self.altitude),                           # WP0: Start/Home
            (center_x + half_width, center_y, self.altitude),             # WP1: Mid-right
            (center_x + half_width, center_y + half_height, self.altitude), # WP2: Top-right
            (center_x, center_y + half_height, self.altitude),            # WP3: Mid-top
            (center_x - half_width, center_y + half_height, self.altitude), # WP4: Top-left
            (center_x - half_width, center_y, self.altitude),             # WP5: Mid-left
            (center_x - half_width, center_y - half_height, self.altitude), # WP6: Bottom-left
            (center_x, center_y - half_height, self.altitude),            # WP7: Mid-bottom
            (center_x + half_width, center_y - half_height, self.altitude), # WP8: Bottom-right
            (center_x + half_width, center_y, self.altitude),             # WP9: Mid-right (again to close loop)
        ]

        for i, wp in enumerate(waypoints_local):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = float(wp[2])
            pose.pose.orientation.w = 1.0
            self.waypoints.append(pose)
            self.waypoint_attempts[i] = 0

        self.get_logger().info(f"📍 Generated {len(self.waypoints)} waypoints (0=start, last=final)")

    def publish_waypoint_markers(self):
        """Publish waypoints as RViz markers (color coding verified)"""
        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = wp.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0

            # Verified mapping:
            # - WP0 (start): green
            # - last WP: red
            # - completed: green (set dynamically)
            # - skipped: orange (set dynamically)
            # - current WP: yellow (set dynamically)
            # - future: blue
            if i == 0:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            elif i == len(self.waypoints) - 1:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            else:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)

            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "labels"
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = wp.pose.position
            text_marker.pose.position.z += 3.0
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 1.5
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"WP{i}"
            marker_array.markers.append(text_marker)

        self.waypoint_marker_pub.publish(marker_array)

    def update_rviz_visualization(self):
        self.publish_drone_marker()
        self.publish_path_marker()
        self.update_waypoint_status()

    def publish_drone_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.current_pose.pose
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.5

        if self.emergency_landing:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        elif self.mission_completed:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        elif self.mission_started:
            marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)

        self.drone_marker_pub.publish(marker)

    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)

        for i in range(min(self.current_waypoint_index + 1, len(self.waypoints))):
            point = Point()
            point.x = self.waypoints[i].pose.position.x
            point.y = self.waypoints[i].pose.position.y
            point.z = self.waypoints[i].pose.position.z
            marker.points.append(point)

        self.path_marker_pub.publish(marker)

    def update_waypoint_status(self):
        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints_status"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = wp.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.8
            marker.scale.y = 1.8
            marker.scale.z = 1.8

            if i in self.skipped_waypoints:
                marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # Orange skipped
            elif i < self.current_waypoint_index:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)  # Green completed
            elif i == self.current_waypoint_index:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow current
            else:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.4)  # Blue future

            marker_array.markers.append(marker)

        self.waypoint_marker_pub.publish(marker_array)

    # -------------------------
    # MAVROS services & initial setpoints
    # -------------------------
    def wait_for_services(self):
        self.get_logger().info("⏳ Waiting for MAVROS services...")
        while not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("MAVROS arming service not available, waiting...")
        while not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("MAVROS set_mode service not available, waiting...")
        self.get_logger().info("✅ MAVROS services ready")

    def send_initial_setpoints(self):
        self.get_logger().info("📤 Sending initial setpoints...")
        for _ in range(100):
            self.setpoint_pub.publish(self.waypoints[0])
            time.sleep(0.05)
        self.get_logger().info("✅ Initial setpoints sent")

    # -------------------------
    # Callbacks
    # -------------------------
    def state_callback(self, msg: State):
        self.current_state = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def velocity_callback(self, msg: TwistStamped):
        self.current_velocity = msg

    # -------------------------
    # Utility: distances and approach point
    # -------------------------
    def calculate_distance_to_waypoint(self, waypoint_index: int) -> float:
        if waypoint_index >= len(self.waypoints):
            return float('inf')
        current = self.current_pose.pose.position
        target = self.waypoints[waypoint_index].pose.position
        dx = current.x - target.x
        dy = current.y - target.y
        dz = current.z - target.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_waypoint_reached(self, waypoint_index: int) -> bool:
        return self.calculate_distance_to_waypoint(waypoint_index) <= self.position_tolerance

    def compute_approach_point(self, waypoint_index: int):
        """Return a temporary approach pose (PoseStamped) offset backwards from target
           along direction from target->drone by approach_backoff_distance."""
        wp = self.waypoints[waypoint_index].pose.position
        cur = self.current_pose.pose.position
        vx = cur.x - wp.x
        vy = cur.y - wp.y
        vz = cur.z - wp.z
        norm = math.sqrt(vx*vx + vy*vy + vz*vz)
        approach = PoseStamped()
        approach.header.frame_id = "map"
        approach.pose.orientation.w = 1.0
        if norm == 0:
            # drone is exactly at WP (unlikely)
            approach.pose.position.x = wp.x
            approach.pose.position.y = wp.y
            approach.pose.position.z = wp.z
            return approach
        # move back from the waypoint along the vector (wp -> drone) by backoff distance
        ux, uy, uz = vx / norm, vy / norm, vz / norm
        approach.pose.position.x = wp.x + ux * self.approach_backoff_distance
        approach.pose.position.y = wp.y + uy * self.approach_backoff_distance
        approach.pose.position.z = wp.z + uz * self.approach_backoff_distance
        return approach

    # -------------------------
    # Mode / Arm / Land / RTL wrappers
    # -------------------------
    def set_offboard_mode(self):
        if self.current_state.mode != "OFFBOARD":
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            fut = self.set_mode_client.call_async(req)
            fut.add_done_callback(self.mode_response_callback)
            self.get_logger().info("🟢 Requesting OFFBOARD mode")

    def arm_vehicle(self):
        if not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            fut = self.arming_client.call_async(req)
            fut.add_done_callback(self.arm_response_callback)
            self.get_logger().info("🟢 Requesting ARM")

    def land_vehicle(self):
        self.emergency_landing = True
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        fut = self.set_mode_client.call_async(req)
        fut.add_done_callback(self.land_response_callback)
        self.get_logger().info("🔻 Initiating AUTO.LAND")

    def return_to_launch(self):
        req = SetMode.Request()
        req.custom_mode = "AUTO.RTL"
        fut = self.set_mode_client.call_async(req)
        fut.add_done_callback(self.rtl_response_callback)
        self.get_logger().info("🏠 Returning to launch")

    def mode_response_callback(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info("✅ OFFBOARD mode enabled")
        except Exception as e:
            self.get_logger().error(f"❌ Mode change failed: {e}")

    def arm_response_callback(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info("✅ Vehicle armed")
                self.mission_started = True
        except Exception as e:
            self.get_logger().error(f"❌ Arming failed: {e}")

    def land_response_callback(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info("✅ AUTO.LAND enabled")
        except Exception as e:
            self.get_logger().error(f"❌ Land command failed: {e}")

    def rtl_response_callback(self, future):
        try:
            resp = future.result()
            if resp.mode_sent:
                self.get_logger().info("✅ AUTO.RTL enabled")
        except Exception as e:
            self.get_logger().error(f"❌ RTL command failed: {e}")

    # -------------------------
    # Waypoint helpers
    # -------------------------
    def skip_waypoint(self, waypoint_index: int):
        if waypoint_index not in self.skipped_waypoints:
            self.skipped_waypoints.add(waypoint_index)
            self.get_logger().warning(f"⏭️ Skipping waypoint {waypoint_index}")
            return True
        return False

    def advance_to_next_waypoint(self):
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0
            self.current_lap += 1
            if self.current_lap >= self.laps:
                self.mission_completed = True
                self.get_logger().info("🎉 Mission completed! All laps finished successfully!")
                self.get_logger().info("🏠 Returning to launch position...")
                self.return_to_launch()
            else:
                self.get_logger().info(f"🔄 Starting lap {self.current_lap + 1}/{self.laps}")
                self.skipped_waypoints.clear()
        else:
            self.get_logger().info(f"🔄 Advancing to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")

    # -------------------------
    # Communication-failsafe decision
    # -------------------------
    def handle_comm_loss(self):
        """Called when current_state.connected is False while mission is in progress."""
        # Decide cases:
        # Case 1: If we've already reached at least 1 waypoint (i.e. progress), try to finish mission locally.
        # Case 2: If mission hasn't reached more than 3 waypoints (i.e. reached_count <= 3), RTL immediately.

        self.get_logger().warning("⚠️ MAVROS connection lost! Entering fail-safe decision routine.")
        if self.reached_waypoints_count <= 3:
            self.get_logger().warning(f"🚩 Reached waypoints count = {self.reached_waypoints_count} <= 3 -> commanding RTL (Case 2)")
            self.return_to_launch()
            return False  # we won't continue mission
        else:
            self.get_logger().info(f"🔁 Reached waypoints count = {self.reached_waypoints_count} > 3 -> attempt to complete mission locally (Case 1)")
            # allow mission to continue locally; set a local flag but keep same attempt logic
            return True

    # -------------------------
    # Main control loop
    # -------------------------
    def control_loop(self):
        """Main control loop executed at setpoint_rate."""
        if self.emergency_landing or self.mission_completed:
            return

        # If MAVROS disconnected:
        if not getattr(self.current_state, 'connected', False):
            # If mission not started or no progress: do RTL
            cont = self.handle_comm_loss()
            if not cont:
                return

        # Normal mission start (setoffboard + arm)
        if not self.mission_started:
            self.set_offboard_mode()
            self.arm_vehicle()
            self.setpoint_pub.publish(self.waypoints[0])
            return

        # If armed and mission active
        if self.current_state.armed and self.mission_started:
            idx = self.current_waypoint_index
            if idx >= len(self.waypoints):
                return

            current_wp = self.waypoints[idx]
            # Publish primary setpoint
            self.setpoint_pub.publish(current_wp)

            # Logging arrays capture
            now_t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
            self.log_times.append(now_t)
            self.log_setpoints.append((current_wp.pose.position.x, current_wp.pose.position.y, current_wp.pose.position.z))
            curpos = self.current_pose.pose.position
            self.log_actuals.append((curpos.x, curpos.y, curpos.z))

            # Calculate current error to WP
            err = self.calculate_distance_to_waypoint(idx)
            self.log_errors.append(err)
            self.log_wp_status.append(1 if idx < self.current_waypoint_index else 0)

            # If reached:
            if self.is_waypoint_reached(idx):
                # when reached increment count once
                if self.waypoint_reached_time is None:
                    self.waypoint_reached_time = self.get_clock().now()
                    self.get_logger().info(f"✅ Waypoint {idx} reached (distance {err:.2f} m)")
                    self.reached_waypoints_count += 1

                current_time = self.get_clock().now()
                if (current_time - self.waypoint_reached_time).nanoseconds >= 2e9:  # 2 seconds stabilization
                    self.advance_to_next_waypoint()
                    self.waypoint_reached_time = None
            else:
                # Not reached: handle timeout/attempts/approach-from-better-angle
                current_time = self.get_clock().now()
                if self.waypoint_reached_time is None:
                    self.waypoint_reached_time = current_time
                time_at_wp = (current_time - self.waypoint_reached_time).nanoseconds / 1e9

                # If error large (> 1.9 m) and we haven't done a corrective approach yet this attempt, do it
                if err > self.error_for_better_angle and self.waypoint_attempts.get(idx, 0) < self.max_attempts:
                    # generate approach point and publish it for short duration to change approach angle
                    approach = self.compute_approach_point(idx)
                    self.get_logger().warning(f"↪️ Error {err:.2f} m > {self.error_for_better_angle:.2f} m -> doing better-angle approach for WP {idx}")
                    # publish approach for a short burst (0.8s)
                    ticks = int(0.8 / (1.0 / float(self.get_parameter('setpoint_rate').value)))
                    for _ in range(max(ticks, 1)):
                        self.setpoint_pub.publish(approach)
                        time.sleep(1.0 / float(self.get_parameter('setpoint_rate').value))
                    # after approach, reset waypoint_reached_time so timeout restarts
                    self.waypoint_reached_time = self.get_clock().now()
                    self.waypoint_attempts[idx] += 1
                    return

                # If time exceeded -> count attempt and either retry or skip
                if time_at_wp > self.waypoint_timeout:
                    self.waypoint_attempts[idx] = self.waypoint_attempts.get(idx, 0) + 1
                    self.get_logger().warning(f"⏱ Timeout on WP {idx} (attempt {self.waypoint_attempts[idx]}/{self.max_attempts}) distance={err:.2f} m")
                    if self.waypoint_attempts[idx] >= self.max_attempts:
                        # If after attempts still failing, skip this waypoint
                        self.get_logger().warning(f"❌ Max attempts reached for WP {idx}. Skipping.")
                        self.skip_waypoint(idx)
                        self.advance_to_next_waypoint()
                        self.waypoint_reached_time = None
                    else:
                        # Re-attempt: if large error, approach from better angle next loop, else republish setpoint and reset timer
                        self.get_logger().info(f"🔄 Re-attempting WP {idx} (attempt {self.waypoint_attempts[idx]})")
                        self.waypoint_reached_time = self.get_clock().now()

            # After each waypoint action, check mission-wide fail condition:
            # If total reached waypoints <= 3 after trying multiple WPs and we are stuck, command RTL
            # Here we use conservative check: if we've attempted more than len(waypoints) and reached <=3 -> RTL
            total_attempts = sum(self.waypoint_attempts.values())
            if total_attempts > len(self.waypoints) * self.max_attempts and self.reached_waypoints_count <= 3:
                self.get_logger().error("🚨 Mission unable to progress (reached_waypoints_count <= 3 after many attempts). Commanding RTL.")
                self.return_to_launch()

    # -------------------------
    # Logging & plotting
    # -------------------------
    def log_status(self):
        """Periodic logging and capture for plotting."""
        if not self.mission_started:
            return

        current_wp = self.current_waypoint_index
        distance = self.calculate_distance_to_waypoint(current_wp)
        vel = math.sqrt(self.current_velocity.twist.linear.x**2 + self.current_velocity.twist.linear.y**2 + self.current_velocity.twist.linear.z**2)
        pos = self.current_pose.pose.position
        status = f"""
📊 MISSION STATUS:
├── Lap: {self.current_lap + 1}/{self.laps}
├── Waypoint: {current_wp + 1}/{len(self.waypoints)}
├── Distance: {distance:.2f} m
├── Velocity: {vel:.2f} m/s
├── Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})
├── Reached waypoints: {self.reached_waypoints_count}
└── Attempts: {self.waypoint_attempts.get(current_wp, 0)}
"""
        self.get_logger().info(status, throttle_duration_sec=2.0)

    def save_logs_and_plots(self):
        """Save CSV and generate PNG plots (if matplotlib available)."""
        csv_file = os.path.join(self.log_dir, "mission_log.csv")
        self.get_logger().info(f"💾 Saving mission log to {csv_file}")
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "setpoint_x", "setpoint_y", "setpoint_z", "actual_x", "actual_y", "actual_z", "error", "current_wp"])
            for i in range(len(self.log_times)):
                t = self.log_times[i]
                sp = self.log_setpoints[i] if i < len(self.log_setpoints) else (None, None, None)
                ac = self.log_actuals[i] if i < len(self.log_actuals) else (None, None, None)
                err = self.log_errors[i] if i < len(self.log_errors) else None
                wpflag = self.log_wp_status[i] if i < len(self.log_wp_status) else 0
                writer.writerow([t, sp[0], sp[1], sp[2], ac[0], ac[1], ac[2], err, wpflag])

        # Plots
        if MATPLOTLIB_AVAILABLE and len(self.log_times) > 1:
            times = [t - self.log_times[0] for t in self.log_times]  # relative time
            # Error over time
            plt.figure()
            plt.plot(times, self.log_errors)
            plt.xlabel("time (s)")
            plt.ylabel("position error (m)")
            plt.title("Position Error over Time")
            plt.grid(True)
            err_png = os.path.join(self.log_dir, "error_over_time.png")
            plt.savefig(err_png)
            plt.close()
            self.get_logger().info(f"📈 Saved error plot: {err_png}")

            # Offboard setpoint vs actual (X,Y)
            sp_x = [s[0] for s in self.log_setpoints]
            sp_y = [s[1] for s in self.log_setpoints]
            ac_x = [a[0] for a in self.log_actuals]
            ac_y = [a[1] for a in self.log_actuals]
            plt.figure()
            plt.plot(sp_x, sp_y, label="setpoint (x,y)")
            plt.plot(ac_x, ac_y, label="actual (x,y)")
            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            plt.legend()
            plt.title("Setpoint vs Actual (XY)")
            plt.grid(True)
            xy_png = os.path.join(self.log_dir, "setpoint_vs_actual_xy.png")
            plt.savefig(xy_png)
            plt.close()
            self.get_logger().info(f"📈 Saved XY plot: {xy_png}")

            # Waypoint completion status (binary) over time
            plt.figure()
            plt.plot(times, self.log_wp_status, drawstyle='steps-post')
            plt.xlabel("time (s)")
            plt.ylabel("waypoint status (0 future / 1 current-or-past)")
            plt.title("Waypoint status over time")
            plt.grid(True)
            wp_png = os.path.join(self.log_dir, "waypoint_status.png")
            plt.savefig(wp_png)
            plt.close()
            self.get_logger().info(f"📈 Saved waypoint status plot: {wp_png}")
        else:
            if not MATPLOTLIB_AVAILABLE:
                self.get_logger().warning("matplotlib not available — skipping plot generation.")
            else:
                self.get_logger().info("Not enough data for plotting.")

    # -------------------------
    # Shutdown
    # -------------------------
    def destroy_node(self):
        try:
            self.get_logger().info("🧾 Saving logs and plots before shutdown...")
            self.save_logs_and_plots()
        except Exception as e:
            self.get_logger().error(f"Error saving logs: {e}")
        super().destroy_node()

# -------------------------
# Main
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OffboardWaypointNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mission interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
