"""Draw the tracker's tracks on the SIMU map. ROS-free.

The tracker works in the EKF's world frame (`map`). With no navsat datum set,
`map` starts wherever the boat got its first GPS fix, so it is NOT the Gazebo
world frame the SIMU map is drawn in. MapAlignment finds the transform
map -> Gazebo world from pairs of (the boat's TRUE Gazebo pose, the boat's EKF
pose) at the same sim time:

    rotation    = circular mean of (true heading - EKF heading)
    translation = mean of (true position - R * EKF position)

over the last `window` pairs. The distance between the true pose and the
aligned EKF pose is shown on the page as the EKF error.
"""

import bisect
import math
from collections import deque


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MapAlignment:
    def __init__(self, window=100, max_gap_s=0.25):
        self.true = deque(maxlen=600)          # (t, x, y, yaw) of the boat in Gazebo
        self.pairs = deque(maxlen=window)      # (true, ekf)
        self.max_gap = max_gap_s
        self.theta, self.tx, self.ty = 0.0, 0.0, 0.0
        self.error = None                      # m, latest |true - aligned EKF|
        self.heading_error = None              # rad

    def add_true(self, t, x, y, yaw):
        if self.true and t < self.true[-1][0] - 1e-6:     # sim reset
            self.true.clear()
            self.pairs.clear()
        if not self.true or t > self.true[-1][0]:
            self.true.append((t, x, y, yaw))

    def true_at(self, t):
        if not self.true:
            return None
        ts = [p[0] for p in self.true]
        i = bisect.bisect_left(ts, t)
        if i == 0:
            p = self.true[0]
            return p[1:] if p[0] - t < self.max_gap else None
        if i == len(ts):
            p = self.true[-1]
            return p[1:] if t - p[0] < self.max_gap else None
        a, b = self.true[i - 1], self.true[i]
        if b[0] - a[0] > 2 * self.max_gap:
            return None
        f = (t - a[0]) / max(b[0] - a[0], 1e-9)
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), wrap(a[3] + f * wrap(b[3] - a[3])))

    def add_ekf(self, t, x, y, yaw):
        tp = self.true_at(t)
        if tp is None:
            return False
        self.pairs.append((tp, (x, y, yaw)))
        s = sum(math.sin(wrap(p[0][2] - p[1][2])) for p in self.pairs)
        c = sum(math.cos(wrap(p[0][2] - p[1][2])) for p in self.pairs)
        self.theta = math.atan2(s, c)
        ct, st = math.cos(self.theta), math.sin(self.theta)
        n = len(self.pairs)
        self.tx = sum(p[0][0] - (ct * p[1][0] - st * p[1][1]) for p in self.pairs) / n
        self.ty = sum(p[0][1] - (st * p[1][0] + ct * p[1][1]) for p in self.pairs) / n
        ax, ay = self.point(x, y)
        self.error = math.hypot(tp[0] - ax, tp[1] - ay)
        self.heading_error = wrap(tp[2] - (yaw + self.theta))
        return True

    @property
    def ready(self):
        return len(self.pairs) > 0

    def point(self, x, y):
        c, s = math.cos(self.theta), math.sin(self.theta)
        return c * x - s * y + self.tx, s * x + c * y + self.ty

    def vector(self, vx, vy):
        c, s = math.cos(self.theta), math.sin(self.theta)
        return c * vx - s * vy, s * vx + c * vy

    def tracks_in_world(self, doc):
        """The tracker's JSON (map frame) -> the same, in Gazebo world coordinates."""
        r = lambda v: round(v, 2)
        out = []
        for tr in doc.get('tracks', []):
            x, y = self.point(tr['x'], tr['y'])
            vx, vy = self.vector(tr['vx'], tr['vy'])
            a, b, ang = tr['ell']
            t = dict(tr)
            t.update(x=r(x), y=r(y), vx=r(vx), vy=r(vy), yaw=round(wrap(tr['yaw'] + self.theta), 3),
                     course=round(wrap(tr['course'] + self.theta), 3), ell=[a, b, round(ang + self.theta, 3)],
                     # [x, y, 2-sigma radius (, ellipse a, b, angle)] -- the angle turns with the frame
                     pred=[[r(px), r(py)] + list(p[2:5]) + ([round(p[5] + self.theta, 3)] if len(p) > 5 else [])
                           for (px, py), p in ((self.point(p[0], p[1]), p) for p in tr.get('pred', []))],
                     trail=[[round(px, 1), round(py, 1)] for px, py in
                            (self.point(p[0], p[1]) for p in tr.get('trail', []))])
            out.append(t)
        return {'stamp': doc.get('stamp'), 'frame': doc.get('frame'), 'horizon': doc.get('horizon', 10.0),
                'tracks': out,
                'align': {'theta_deg': round(math.degrees(self.theta), 2), 'tx': round(self.tx, 2),
                          'ty': round(self.ty, 2), 'pairs': len(self.pairs),
                          'ekf_error_m': None if self.error is None else round(self.error, 2),
                          'ekf_heading_error_deg': (None if self.heading_error is None
                                                    else round(math.degrees(self.heading_error), 2))}}
