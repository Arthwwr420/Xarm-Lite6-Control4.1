import numpy as np

waypoints = [
    (0.275,  0.05, 0.25),
    (0.275,  0.05, 0.05),
    (0.275,  0.05, 0.25),

    (0.375,  0.05, 0.25),
    (0.375,  0.05, 0.05),
    (0.375,  0.05, 0.25),

    (0.375, -0.05, 0.25),
    (0.375, -0.05, 0.05),
    (0.375, -0.05, 0.25),

    (0.275, -0.05, 0.25),
    (0.275, -0.05, 0.05),
    (0.275, -0.05, 0.25),
]

class MyTrajectory:
    def __init__(self, dwell_sec=1.0, segment_sec=2.0, loop=True):
        self.waypoints = [np.array(w, dtype=float) for w in waypoints]
        self.dwell_sec = dwell_sec
        self.segment_sec = segment_sec
        self.loop = loop
        self.num_segments = len(self.waypoints) - 1

    def sample(self, t):
        block = self.dwell_sec + self.segment_sec
        total = self.num_segments * block

        if self.loop:
            t = t % total
        else:
            t = min(t, total - 1e-9)

        seg = min(int(t // block), self.num_segments - 1)
        tau = t - seg * block

        p0 = self.waypoints[seg]
        p1 = self.waypoints[seg + 1]

        if tau < self.dwell_sec:
            p_des = p0.copy()
            p_dot_des = np.zeros(3)
            p_ddot_des = np.zeros(3)
            phase = "dwell"
        else:
            s = (tau - self.dwell_sec) / self.segment_sec
            s = np.clip(s, 0.0, 1.0)

            a = 3*s**2 - 2*s**3
            da = (6*s - 6*s**2) / self.segment_sec
            dda = (6 - 12*s) / (self.segment_sec**2)

            dp = p1 - p0
            p_des = p0 + a * dp
            p_dot_des = da * dp
            p_ddot_des = dda * dp
            phase = "move"

        return p_des, p_dot_des, p_ddot_des, seg, phase