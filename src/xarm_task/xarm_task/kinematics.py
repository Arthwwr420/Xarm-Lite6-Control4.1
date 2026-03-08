import numpy as np

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

DH = [
    (0.0,           0.2433,   -np.pi/2,  0.0),
    (-np.pi/2,      0.0,       np.pi,    0.200),
    (-np.pi/2,      0.0,       np.pi/2,  0.087),
    (0.0,           0.2276,     np.pi/2,  0.0),
    (0.0,           0.0,       -np.pi/2,  0.0),
    (0.0,           0.0615,     0.0,      0.0),
]

def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,   d],
        [0,       0,      0,   1],
    ], dtype=float)

def fk_all(q: np.ndarray) -> list[np.ndarray]:
    T = np.eye(4)
    Ts = []
    for i in range(6):
        th_off, d, alpha, a = DH[i]
        T = T @ dh_matrix(q[i] + th_off, d, a, alpha)
        Ts.append(T.copy())
    return Ts

def fk_T06(q: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    for i in range(6):
        th_off, d, alpha, a = DH[i]
        T = T @ dh_matrix(q[i] + th_off, d, a, alpha)
    return T

def ee_pos(q: np.ndarray) -> np.ndarray:
    return fk_T06(q)[:3, 3].copy()

def jacobian_pos(q: np.ndarray) -> np.ndarray:
    Ts = fk_all(q)
    p_e = Ts[-1][:3, 3]

    p_prev = np.array([0.0, 0.0, 0.0])
    z_prev = np.array([0.0, 0.0, 1.0])

    Jv = np.zeros((3, 6))
    for i in range(6):
        if i > 0:
            p_prev = Ts[i-1][:3, 3]
            z_prev = Ts[i-1][:3, 2]
        Jv[:, i] = np.cross(z_prev, (p_e - p_prev))
    return Jv

def forward_kinematics(q: np.ndarray):
    T = fk_T06(q)
    p = T[:3, 3].copy()
    return p, T