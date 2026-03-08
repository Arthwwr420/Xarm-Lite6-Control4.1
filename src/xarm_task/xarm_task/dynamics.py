import numpy as np
from .kinematics import fk_all

masses = np.array([1.411, 1.34, 0.953, 1.284, 0.804, 0.13], dtype=float)

com = np.array([
    [-0.00036,  0.04195, -0.0025],
    [ 0.1790,   0.0,      0.0584],
    [ 0.0720,  -0.0357,  -0.0010],
    [-0.0020,  -0.0285,  -0.0813],
    [ 0.0,      0.01,     0.0019],
    [ 0.0,     -0.00194, -0.0102],
], dtype=float)

def smooth_sign(x: np.ndarray, eps: float = 1e-3) -> np.ndarray:
    return np.tanh(x / eps)

class Lite6NominalDynamics:
    def M0(self, q: np.ndarray) -> np.ndarray:
        base = 0.8 + 2.5 * (masses / masses.max())
        diag = base + 0.25 * np.cos(q)
        diag = np.maximum(diag, 0.2)
        return np.diag(diag)

    def C0_times_qd(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        return 0.12 * qd + 0.03 * np.sin(q) * np.roll(qd, 1)

    def G0(self, q: np.ndarray) -> np.ndarray:
        g = 9.81
        Ts = fk_all(q)
        G = np.zeros(6)
        for i in range(6):
            z_i = Ts[i][:3, 2]
            G[i] = masses[i] * g * z_i[2] * 0.08
        return G

    def F0(self, qd: np.ndarray) -> np.ndarray:
        visc = np.array([0.6, 0.55, 0.45, 0.35, 0.25, 0.18])
        coul = np.array([0.35, 0.32, 0.28, 0.20, 0.16, 0.10])
        return visc * qd + coul * np.sign(qd)

_nom = Lite6NominalDynamics()

def get_dynamics(q: np.ndarray, qd: np.ndarray):
    M = _nom.M0(q)
    Cqd = _nom.C0_times_qd(q, qd)
    G = _nom.G0(q)
    F = _nom.F0(qd)
    return M, Cqd, G, F