import math

class Straight:
    def __init__(self, l): self.l = l
class Curve:
    def __init__(self, r, a): self.r = r; self.a = a

segs = [
    Straight(1.0),
    Curve(1.0, 90),
    Curve(1.0, -90),
    Straight(1.0),
    Curve(1.0, 180),
    Straight(1.0),
    Curve(1.0, 90),
    Curve(1.0, -90),
    Straight(1.0),
    Curve(1.0, 90),
    Curve(1.0, 90),
]

x, y, h = 0, 0, 0
for s in segs:
    if isinstance(s, Straight):
        x += s.l * math.cos(h)
        y += s.l * math.sin(h)
    else:
        rad = math.radians(s.a)
        cx = x + s.r * math.cos(h + math.pi/2 * (1 if s.a > 0 else -1))
        cy = y + s.r * math.sin(h + math.pi/2 * (1 if s.a > 0 else -1))
        h += rad
        x = cx - s.r * math.cos(h + math.pi/2 * (1 if s.a > 0 else -1))
        y = cy - s.r * math.sin(h + math.pi/2 * (1 if s.a > 0 else -1))
print(f"End: x={x:.3f}, y={y:.3f}, h={math.degrees(h)%360:.1f}")
