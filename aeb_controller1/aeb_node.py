#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarTrackList
from inertial_msgs.msg import Pose
from vehiclecontrol.msg import Control
from geometry_msgs.msg import Point
import numpy as np

class AEBControllerNode(Node):
    def __init__(self):
        super().__init__('aeb_controller1_radar_only')

        self.get_logger().info("AEBControllerNode initialization started.")

        # Publishers/Subscribers
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        self.radar_sub = self.create_subscription(
            RadarTrackList, '/RadarObjects', self.radar_callback, 10)
        self.pose_sub = self.create_subscription(
            Pose, '/InertialData', self.pose_callback, 10)

        # Parameters
        self.max_throttle = 1.0
        self.max_brake = 0.75
        self.target_speed = 8.33333   # 30 km/h in m/s
        self.target_stop_distance = 6.0
        self.stop_tolerance = 2.0
        self.min_decel = 5.0
        self.max_decel = 8.0
        self.safe_ttc_threshold = 2.5
        self.emergency_ttc = 1.0
        self.warning_ttc = 3.0
        self.vehicle_width = 1.5
        self.target_width_car = 1.8
        self.target_width_bicycle = 0.7
        
        # State
        self.ego_velocity = 0.0
        self.prev_ego_velocity = 0.0
        self.ego_accel = 0.0
        self.distance_traveled = 0.0
        self.last_time = self.get_clock().now()
        self.current_distance = float('inf')
        self.relative_velocity = 0.0
        self.dt = 0.1
        self.current_ttc = float('inf')
        self.target_position = Point()
        self.stopped = False   # Latch for stop state
        self.braking_latch = False

        # ---- Latch for AEB stop ----
        self.latched_stop = False

        self.get_logger().info("AEBControllerNode initialized.")

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if abs(dt) > 0:
            self.dt = abs(dt)
        else:
            self.dt = 0.1  # Fallback in case of zero dt

        self.prev_ego_velocity = abs(self.ego_velocity)
        self.ego_velocity = abs(msg.velocity.x)
        self.ego_accel = (self.ego_velocity - self.prev_ego_velocity) / self.dt
        self.distance_traveled += self.ego_velocity * self.dt

        # LOGGING: Inertial data, current velocity, and deceleration/acceleration
        self.get_logger().info(
            f"[INERTIAL] dt={self.dt:.3f}s, Prev V_ego={self.prev_ego_velocity:.2f} m/s, "
            f"V_ego={self.ego_velocity:.2f} m/s, a_ego={self.ego_accel:.2f} m/s², "
            f"Distance traveled={self.distance_traveled:.2f} m"
        )

    def radar_callback(self, msg):
        self.get_logger().debug("[RADAR] Received radar message.")
        try:
            closest_obj = None
            min_distance = float('inf')

            for obj in msg.objects:
                if abs(obj.y_distance) < 3.0 and 0 < obj.x_distance < min_distance:
                    min_distance = obj.x_distance
                    closest_obj = obj

            if closest_obj:
                self.current_distance = closest_obj.x_distance
                # vx is usually the relative velocity in radar, negative means closing
                self.relative_velocity = -closest_obj.vx
                self.target_position = Point(
                    x=closest_obj.x_distance,
                    y=closest_obj.y_distance,
                    z=0.0
                )
                if self.relative_velocity > 0.1:
                    self.current_ttc = self.current_distance / self.relative_velocity
                else:
                    self.current_ttc = float('inf')
                self.get_logger().info(
                    f"[RADAR] Closest object: d={self.current_distance:.2f}m, y={closest_obj.y_distance:.2f}, rel_v={self.relative_velocity:.2f} m/s, TTC={self.current_ttc:.2f} s"
                )
            else:
                self.current_distance = float('inf')
                self.relative_velocity = 0.0
                self.current_ttc = float('inf')
                self.get_logger().info("[RADAR] No relevant object found.")

            self.execute_control()
        except Exception as e:
            self.get_logger().error(f"Radar processing error: {str(e)}")


    def execute_control(self):
        control_msg = Control()
        control_msg.steering = 0.0
        control_msg.latswitch = 0
        control_msg.longswitch = 1

        # -- Latch check --
        if self.latched_stop:
            control_msg.throttle = 0.0
            control_msg.brake = self.max_brake
            self.publish_control(control_msg)
            self.get_logger().info("[AEB LATCH] Holding stop: Brake=100%, Throttle=0% due to previous AEB stop.")
            return

        # Log current status (including acceleration/deceleration)
        self.get_logger().info(
            f"[CONTROL] V_ego={self.ego_velocity:.2f}m/s, a_ego={self.ego_accel:.2f}m/s², d_traveled={self.distance_traveled:.2f}m, "
            f"d_obj={self.current_distance:.2f}m, ttc={self.current_ttc:.2f}s"
        )

        # --- TTC BASED BRAKING ---
        TTC_BRAKE_THRESHOLD = 1.5  # seconds

        if 0 < self.current_ttc < TTC_BRAKE_THRESHOLD:
        # --- Map required deceleration to brake command (safe AEB) ---
            v = max(self.ego_velocity, 0.1)  # Prevent divide by zero
            d = max(self.current_distance - self.target_stop_distance, 0.1)  # distance to stop
            required_decel = (v ** 2) / (2 * d)
            clamped_decel = np.clip(required_decel, self.min_decel, self.max_decel)
            brake_percent = (clamped_decel - self.min_decel) / (self.max_decel - self.min_decel)
            brake_percent = np.clip(brake_percent, 0.01, 1.0)
            control_msg.throttle = 0.0
            control_msg.brake = brake_percent * self.max_brake
            self.latched_stop = True
            
            self.publish_control(control_msg)
            self.get_logger().warn(
                f"[AEB BRAKE] TTC={self.current_ttc:.2f}s < {TTC_BRAKE_THRESHOLD}s: "
                f"ReqDecel={required_decel:.2f} m/s², Clamped={clamped_decel:.2f} m/s², "
                f"Brake Command={control_msg.brake:.1f}%"
            )

        # -- Set latch if stopped in the required zone --
            front_gap = self.current_distance
            stop_min = self.target_stop_distance - self.stop_tolerance  # 4m
            stop_max = self.target_stop_distance + self.stop_tolerance  # 8m
            if abs(self.ego_velocity) < 0.1 and stop_min <= front_gap <= stop_max:
                self.latched_stop = True
                self.get_logger().info("[AEB LATCH] Vehicle stopped in 4-8m gap. Latch engaged.")
            return

        # Zone 1: Acceleration (keep as fallback)
        if self.distance_traveled < 100.0 and self.ego_velocity < self.target_speed:
            control_msg.throttle = self.max_throttle
            control_msg.brake = 0.0
            self.publish_control(control_msg)
            self.get_logger().info(f"Zone 1: Accelerating. Throttle: {control_msg.throttle:.2f}%")
            return
        # Clamp speed above target
        if self.ego_velocity > self.target_speed + 0.5:
            v = self.ego_velocity
            v_target = self.target_speed
            dt = max(self.dt, 0.01)
            # Estimate required decel to reach target speed in dt seconds
            required_decel = (v - v_target) / dt
            clamped_decel = np.clip(required_decel, self.min_decel, self.max_decel)
            brake_percent = (clamped_decel - self.min_decel) / (self.max_decel - self.min_decel)
            brake_percent = np.clip(brake_percent, 0.01, 1.0)
            control_msg.throttle = 0.0
            control_msg.brake = brake_percent * self.max_brake
            self.publish_control(control_msg)
            self.latched_stop = True
            self.get_logger().info(
                f"[CLAMP] Speed > target. Decel: {required_decel:.2f} m/s² (clamped {clamped_decel:.2f}), "
                f"Brake: {control_msg.brake:.1f}%"
            )
            return
                
        # Default: Coasting
        control_msg.throttle = 0.0
        control_msg.brake = 0.0
        self.publish_control(control_msg)
        self.get_logger().info("[DEFAULT] No action needed. Coasting.")

    def publish_control(self, control_msg):
        self.get_logger().info(
            f"Publishing Control Command - Steering: {control_msg.steering}, "
            f"Throttle: {control_msg.throttle}, Brake: {control_msg.brake}, "
            f"LongSwitch: {control_msg.longswitch}, LatSwitch: {control_msg.latswitch}"
        )
        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBControllerNode()
    node.get_logger().info("AEBControllerNode spinning. Waiting for data...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AEBControllerNode interrupted by user (KeyboardInterrupt).")
    finally:
        node.get_logger().info("AEBControllerNode shutting down, cleaning up.")
        node.destroy_node()
        rclpy.shutdown()
        print("AEBControllerNode terminated.")

if __name__ == '__main__':
    main()
