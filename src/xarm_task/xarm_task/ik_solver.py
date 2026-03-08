import numpy as np
from .kinematics import ee_pos, jacobian_pos

def clamp(x: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)

class WeightedIKSolver:
    def __init__(self, wz=2.5, lam=1.5e-2, k_task=14.0, k_null=1.5, q_home=None, q_limits=None):
        self.wz = float(wz)
        self.lam = float(lam)
        self.k_task = float(k_task)
        self.k_null = float(k_null)
        self.q_home = np.zeros(6) if q_home is None else np.array(q_home, dtype=float)
        self.q = self.q_home.copy()
        self.qd_prev = np.zeros(6)
        self.J_prev = None
        self.q_limits = q_limits

    def step(self, dt, p_des, p_dot_des, p_ddot_des):
        q = self.q
        W = np.diag([1.0, 1.0, self.wz])

        p = ee_pos(q)
        J = jacobian_pos(q)
        Jw = W @ J

        e_p = np.array(p_des, dtype=float) - p
        v_task = W @ (np.array(p_dot_des, dtype=float) + self.k_task * e_p)

        A = Jw @ Jw.T + (self.lam ** 2) * np.eye(3)
        Jpinv = Jw.T @ np.linalg.inv(A)

        Nproj = np.eye(6) - Jpinv @ Jw
        q_null = -self.k_null * (q - self.q_home)

        qd = Jpinv @ v_task + Nproj @ q_null

        if self.J_prev is not None:
            Jdot = (J - self.J_prev) / dt
            Jwdot = W @ Jdot
            a_task = W @ (np.array(p_ddot_des, dtype=float) + self.k_task * (np.array(p_dot_des, dtype=float) - J @ qd))
            qdd = Jpinv @ (a_task - Jwdot @ qd)
        else:
            qdd = np.zeros(6)

        q_des = q.copy()
        qd_des = qd.copy()
        qdd_des = qdd.copy()

        q_next = q + qd * dt
        if self.q_limits is not None:
            qmin, qmax = self.q_limits
            q_next = clamp(q_next, qmin, qmax)

        self.q = q_next
        self.qd_prev = qd
        self.J_prev = J

        return q_des, qd_des, qdd_des