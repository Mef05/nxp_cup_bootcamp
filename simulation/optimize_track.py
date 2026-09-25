import math
import matplotlib.pyplot as plt

class Straight:
    def __init__(self, l): self.l = l
class Curve:
    def __init__(self, r, a): self.r = r; self.a = a

def calc_track(l1, l2, l3, l4):
    segs = [
        Straight(l1),         # E
        Curve(0.6, 90),       # L90 -> N
        Curve(0.6, -90),      # R90 -> E (S-curve)
        Straight(l2),         # E
        Curve(0.8, 180),      # L180 Hairpin -> W
        Straight(l3),         # W
        Curve(0.4, 45),       # L45
        Curve(0.4, -45),      # R45 (Chicane)
        Straight(l4),         # W
        Curve(1.0, 90),       # L90 -> S
        Curve(1.0, 90)        # L90 -> E (back to start)
    ]
    x, y, h = 0, 0, 0
    pts = [(x,y)]
    for s in segs:
        if isinstance(s, Straight):
            n = 10
            for _ in range(n):
                x += s.l/n * math.cos(h)
                y += s.l/n * math.sin(h)
                pts.append((x,y))
        else:
            rad = math.radians(s.a)
            cx = x + s.r * math.cos(h + math.pi/2 * (1 if s.a > 0 else -1))
            cy = y + s.r * math.sin(h + math.pi/2 * (1 if s.a > 0 else -1))
            n = 20
            dh = rad/n
            for _ in range(n):
                h += dh
                x = cx - s.r * math.cos(h + math.pi/2 * (1 if s.a > 0 else -1))
                y = cy - s.r * math.sin(h + math.pi/2 * (1 if s.a > 0 else -1))
                pts.append((x,y))
    return x, y, pts

from scipy.optimize import minimize
def loss(p):
    x, y, _ = calc_track(*p)
    return x**2 + y**2

res = minimize(loss, [1.0, 1.0, 1.0, 1.0], bounds=[(0.1, 5)]*4)
print(f"L1={res.x[0]:.3f}, L2={res.x[1]:.3f}, L3={res.x[2]:.3f}, L4={res.x[3]:.3f}")

_, _, pts = calc_track(*res.x)
xs = [p[0] for p in pts]
ys = [p[1] for p in pts]
print(f"Max X: {max(xs)}, Min X: {min(xs)}")
print(f"Max Y: {max(ys)}, Min Y: {min(ys)}")
