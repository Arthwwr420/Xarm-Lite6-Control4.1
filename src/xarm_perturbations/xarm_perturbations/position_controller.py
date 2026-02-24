#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener


class CartesianPIDController(Node):
    """
    Cartesian XYZ PID based on TF error between base_frame and ee_frame.
    Publishes TwistStamped to MoveIt Servo delta_twist_cmds.
    """

    def __init__(self):
        super().__init__("cartesian_pid_controller")

        # ---------------------------
        # Params (matching your CLI style)
        # ---------------------------
        self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds")
        self.declare_parameter("base_frame", "link_base")
        self.declare_parameter("ee_frame", "link_eef")

        self.declare_parameter("target_xyz", [0.227, 0.00, 0.468])  # safe default/home-ish
        self.declare_parameter("kp", [2.5, 2.5, 2.5])
        self.declare_parameter("kd", [0.6, 0.6, 0.6])
        self.declare_parameter("ki", [0.0, 0.0, 0.0])

        self.declare_parameter("deadband", 0.002)  # meters
        self.declare_parameter("max_speed", 0.10)  # m/s per axis
        self.declare_parameter("i_clamp", 0.20)    # integrator clamp (m*s)
        self.declare_parameter("rate_hz", 50.0)

        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.target_xyz = np.array(self.get_parameter("target_xyz").value, dtype=float)

        self.kp = np.array(self.get_parameter("kp").value, dtype=float)
        self.kd = np.array(self.get_parameter("kd").value, dtype=float)
        self.ki = np.array(self.get_parameter("ki").value, dtype=float)

        self.deadband = float(self.get_parameter("deadband").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.i_clamp = float(self.get_parameter("i_clamp").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        # ---------------------------
        # Pub + TF (same style as your heart.py)
        # ---------------------------
        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------
        # PID state
        # ---------------------------
        self.prev_error = np.zeros(3, dtype=float)
        self.i_term = np.zeros(3, dtype=float)
        self.prev_time = self.get_clock().now()
        self.last_warn_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._loop)

        self.get_logger().info(
            f"CartesianPIDController OK\n"
            f" base_frame={self.base_frame} ee_frame={self.ee_frame}\n"
            f" target_xyz={self.target_xyz.round(3)}\n"
            f" output_topic={self.output_topic}\n"
            f" kp={self.kp} ki={self.ki} kd={self.kd}\n"
            f" deadband={self.deadband} max_speed={self.max_speed} i_clamp={self.i_clamp}"
        )

    def _read_pose(self):
        """
        Read EE position in base_frame using TF.
        Uses the SAME call pattern as your working heart.py.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            return np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=float,
            )
        except Exception as e:
            now = self.get_clock().now()
            # throttle warnings
            if (now - self.last_warn_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_warn_time = now
            return None

    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = float(v_xyz[0])
        cmd.twist.linear.y = float(v_xyz[1])
        cmd.twist.linear.z = float(v_xyz[2])
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.pub.publish(cmd)

    def _loop(self):
        current = self._read_pose()
        if current is None:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6

        error = self.target_xyz - current

        # Deadband per axis
        mask = np.abs(error) >= self.deadband
        error_db = np.where(mask, error, 0.0)

        d_error = (error_db - self.prev_error) / dt

        # Candidate integrator update
        i_next = self.i_term + error_db * dt
        i_next = np.clip(i_next, -self.i_clamp, self.i_clamp)

        # PID
        v = self.kp * error_db + self.kd * d_error + self.ki * i_next

        # Saturation per-axis (same spirit as your heart.py)
        v_sat = np.clip(v, -self.max_speed, self.max_speed)

        # Anti-windup: if an axis saturates, freeze its integrator update
        saturated_axes = (np.abs(v) > self.max_speed + 1e-12)
        i_next = np.where(saturated_axes, self.i_term, i_next)

        self.i_term = i_next
        self.prev_error = error_db
        self.prev_time = now

        self._publish_twist(v_sat)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()