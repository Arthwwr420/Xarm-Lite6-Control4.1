#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
import rclpy.time
import matplotlib.pyplot as plt


class Plotter(Node):
    def __init__(self):
        super().__init__("plotter_logger")

        self.csv_path = "Results/trajectory_log.csv"
        self.base_frame = "link_base"
        self.ee_frame = "link_eef"

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t0 = self.get_clock().now()

        self.f = open(self.csv_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_sec", "tx", "ty", "tz", "x", "y", "z"])
        self.f.flush()

        self.sub = self.create_subscription(
            Float32MultiArray, "/target_pos", self._on_target, 50
        )

        self.xs = []
        self.ys = []

        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], marker="o", markersize=2, linewidth=1, label="EE (real)")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Trayectoria XY del end-effector")
        self.ax.grid(True)
        self.ax.axis("equal")
        self.ax.legend()

        self.plot_timer = self.create_timer(0.2, self._update_plot)

        self.get_logger().info(
            f"Logging /target_pos + TF({self.base_frame}->{self.ee_frame}) to {self.csv_path}"
        )

    def _read_ee(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            return (
                float(trans.transform.translation.x),
                float(trans.transform.translation.y),
                float(trans.transform.translation.z),
            )
        except Exception:
            return (None, None, None)

    def _on_target(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return

        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        tx, ty, tz = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        x, y, z = self._read_ee()

        self.w.writerow([t, tx, ty, tz, x, y, z])
        self.f.flush()

        if x is not None and y is not None:
            self.xs.append(x)
            self.ys.append(y)

    def _update_plot(self):
        if len(self.xs) < 2:
            return
        self.line.set_data(self.xs, self.ys)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def destroy_node(self):
        try:
            plt.ioff()
            plt.savefig("Results/ee_xy.png", dpi=300)
        except Exception:
            pass
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()