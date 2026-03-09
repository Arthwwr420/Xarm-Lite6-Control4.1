#!/usr/bin/env python3
import ast
import csv
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener

from .perforador_prisma import MyTrajectory
from .ik_solver import WeightedIKSolver
from .kinematics import JOINT_NAMES, forward_kinematics
from .dynamics import get_dynamics
from std_msgs.msg import Float32MultiArray
import rclpy.time



def _parse_list_param(x, expected_len=None):
    if isinstance(x, str):
        x = ast.literal_eval(x)
    if isinstance(x, (list, tuple, np.ndarray)):
        arr = np.array(x, dtype=float).reshape(-1)
        if expected_len is not None and arr.size != expected_len:
            return None
        return arr
    return None


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.ctrl_type = str(self.declare_parameter('controller_type', 'CTC').value).upper()
        self.loop_traj = bool(self.declare_parameter('loop_trajectory', True).value)

        self.rate_hz = float(self.declare_parameter('rate_hz', 200.0).value)
        self.dt = 1.0 / max(1.0, self.rate_hz)

        self.vel_limit = float(self.declare_parameter('vel_limit', np.pi).value)
        self.torque_limit = float(self.declare_parameter('torque_limit', 10.0).value)

        self.save_csv = bool(self.declare_parameter('save_csv', True).value)
        self.csv_dir = str(self.declare_parameter('csv_dir', 'Results').value)

        self.logging_enabled = False
        self.log_enable_threshold = float(self.declare_parameter('log_enable_threshold', 0.02).value)

        # Gains (6 joints)
        kp6 = _parse_list_param(self.declare_parameter('kp6', [2000,2000,2000,2250,2000,1600]).value, 6)
        kd6 = _parse_list_param(self.declare_parameter('kd6', [90, 90, 90, 95, 90, 80]).value, 6)
        ki6 = _parse_list_param(self.declare_parameter('ki6', [500, 500, 500,  560,  500,  400]).value, 6)

        self.Kp = np.diag(kp6)
        self.Kd = np.diag(kd6)
        self.Ki = np.diag(ki6)
        self.i_limit = float(self.declare_parameter('i_limit', 0.6).value)
        self.e_int = np.zeros(6)

        # CTC gains
        kp_ctc = _parse_list_param(self.declare_parameter('kp_ctc', [1600,1600,1600,2250,1600,1600]).value, 6)
        kd_ctc = _parse_list_param(self.declare_parameter('kd_ctc', [20, 20, 20, 14,  12,  10]).value, 6)
        self.Kp_ctc = np.diag(kp_ctc)
        self.Kd_ctc = np.diag(kd_ctc)

        self.gamma = float(self.declare_parameter('gamma', 2.5).value)
        self.k_robust = float(self.declare_parameter('k_robust', 5.5).value)
        self.sign_eps = float(self.declare_parameter('sign_eps', 2e-3).value)

        # IK params
        wz = float(self.declare_parameter('wz', 2.5).value)
        lam = float(self.declare_parameter('lam', 1.5e-3).value)
        k_task = float(self.declare_parameter('k_task', 25.0).value)
        k_null = float(self.declare_parameter('k_null', 0.5).value)
        self.ik = WeightedIKSolver(wz=wz, lam=lam, k_task=k_task, k_null=k_null, q_home=np.zeros(6))

        # Trajectory
        self.dwell_sec = float(self.declare_parameter('dwell_sec', 1.0).value)
        self.segment_sec = float(self.declare_parameter('segment_sec', 3.0).value)
        self.traj = MyTrajectory(
            dwell_sec=self.dwell_sec,
            segment_sec=self.segment_sec,
            loop=self.loop_traj
        )

        # State
        self.q = None
        self.qd = None
        self._joint_map = None
        self.stopped = False
        self.t0 = self.get_clock().now()

        # TF2 (solo para log)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_js = self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.sub_stop = self.create_subscription(Bool, '/challenge_stop', self._on_stop, 10)
        self.pub_joint = self.create_publisher(JointJog, '/servo_server/delta_joint_cmds', 10)
        self.pub_target = self.create_publisher(Float32MultiArray, '/target_pos', 10)


        # CSV
        self.csv_f = None
        self.csv_w = None
        if self.save_csv:
            os.makedirs(self.csv_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self.csv_path = os.path.join(self.csv_dir, f"trial_{self.ctrl_type.lower()}_{stamp}.csv")
            self.csv_f = open(self.csv_path, 'w', newline='')
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow([
                "t","phase","wp_idx",
                *[f"q{i+1}" for i in range(6)],
                *[f"qd{i+1}" for i in range(6)],
                *[f"qdes{i+1}" for i in range(6)],
                *[f"qd_des{i+1}" for i in range(6)],
                *[f"qd_cmd{i+1}" for i in range(6)],
                "px","py","pz",
                "px_des","py_des","pz_des",
                "ex","ey","ez","e_norm",
            ])

        self.timer = self.create_timer(self.dt, self._loop)
        self.get_logger().info(f"Controller listo. type={self.ctrl_type} rate={self.rate_hz}Hz")
        self.get_logger().info(f"Papu:. kp={kp6[0]},{kp6[1]},{kp6[2]},{kp6[3]},{kp6[4]},{kp6[5]} rate={self.rate_hz}Hz")


    def _on_stop(self, msg: Bool):
        if msg.data:
            self.stopped = True
            self.get_logger().warn("E-STOP recibido. Deteniendo.")

    def _on_js(self, msg: JointState):
        if self._joint_map is None:
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            idxs = []
            ok = True
            for jn in JOINT_NAMES:
                if jn in name_to_idx:
                    idxs.append(name_to_idx[jn])
                else:
                    ok = False
                    break
            self._joint_map = idxs if ok else list(range(min(6, len(msg.position))))
            if ok:
                self.get_logger().info(f"JOINT_NAMES OK: {JOINT_NAMES}")
            else:
                self.get_logger().warn("JOINT_NAMES no match. Usando primeros 6 joints de /joint_states.")

        idx = self._joint_map
        self.q = np.array([msg.position[i] for i in idx], dtype=float)
        if msg.velocity and len(msg.velocity) >= max(idx) + 1:
            self.qd = np.array([msg.velocity[i] for i in idx], dtype=float)
        else:
            self.qd = np.zeros(6)

    @staticmethod
    def _smooth_sign(x, eps):
        return np.tanh(x / max(1e-9, eps))

    def _eef_pos_tf2(self):
        try:
            tf = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z], dtype=float)
        except Exception:
            return None

    def _publish_jointjog(self, qd_cmd):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        msg.velocities = [float(x) for x in qd_cmd]
        msg.duration = float(self.dt)
        self.pub_joint.publish(msg)

    def _publish_target(self,target: np.ndarray):
        msg = Float32MultiArray()
        msg.data = target.tolist()
        self.pub_target.publish(msg)


    def _loop(self):
        if self.stopped:
            self._publish_jointjog(np.zeros(6))
            return

        if self.q is None or self.qd is None:
            return
        
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        p_des, p_dot_des, p_ddot_des, wp_idx, phase = self.traj.sample(t)
        self._publish_target(p_des)

        q_des, qd_des, qdd_des = self.ik.step(self.dt, p_des, p_dot_des, p_ddot_des)

        e = self.q - q_des
        e_dot = self.qd - qd_des

        if self.ctrl_type == "PID":
            self.e_int = np.clip(self.e_int + e * self.dt, -self.i_limit, self.i_limit)
            qdd_cmd = qdd_des - (self.Kp @ e + self.Kd @ e_dot + self.Ki @ self.e_int)
            qd_cmd = qd_des + qdd_cmd * self.dt

        elif self.ctrl_type == "PD":
            qdd_cmd = qdd_des - (self.Kp @ e + self.Kd @ e_dot)
            qd_cmd = qd_des + qdd_cmd * self.dt

        elif self.ctrl_type == "CTC":
            M, Cqd, G, F = get_dynamics(self.q, self.qd)
            v = qdd_des - (self.Kp_ctc @ e + self.Kd_ctc @ e_dot)
            S = self.gamma * e + e_dot
            tau = (M @ v + Cqd + G + F) + self.k_robust * self._smooth_sign(S, self.sign_eps)
            tau = np.clip(tau, -self.torque_limit, self.torque_limit)
            qdd_cmd = v  # como M=I en dynamics.py
            qd_cmd = qd_des + qdd_cmd * self.dt

        else:
            self.get_logger().error(f"controller_type invalido: {self.ctrl_type}")
            return

        qd_cmd = np.clip(qd_cmd, -self.vel_limit, self.vel_limit)
        self._publish_jointjog(qd_cmd)

        p_act = self._eef_pos_tf2()
        if p_act is None:
            p_act, _ = forward_kinematics(self.q)

        e_xyz = p_act - p_des
        e_norm = np.linalg.norm(e_xyz)

        if not self.logging_enabled:
            if e_norm < self.log_enable_threshold:
                self.logging_enabled = True
                self.get_logger().info(
                    f"Logging habilitado en t={t:.2f}s, e_norm={e_norm:.4f} m"
                )
            else:
                return

        if self.csv_w is not None:
            self.csv_w.writerow([
                t, phase, wp_idx,
                *self.q.tolist(),
                *self.qd.tolist(),
                *q_des.tolist(),
                *qd_des.tolist(),
                *qd_cmd.tolist(),
                *p_act.tolist(),
                *p_des.tolist(),
                float(e_xyz[0]), float(e_xyz[1]), float(e_xyz[2]), float(e_norm),
            ])
            self.csv_f.flush()           

    def destroy_node(self):
        if self.csv_f is not None:
            self.csv_f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()