"""Pure polygon operations for finite above-target footprints."""


def in_ring(x, y, ring):
    inside = False
    for a,b in zip(ring,(*ring[1:],ring[0])):
        ax,ay,bx,by = *a[:2],*b[:2]
        cross = (x-ax)*(by-ay)-(y-ay)*(bx-ax)
        if abs(cross) < 1e-12 and min(ax,bx)-1e-12 <= x <= max(ax,bx)+1e-12 and min(ay,by)-1e-12 <= y <= max(ay,by)+1e-12:
            return True
        if (ay > y) != (by > y) and x < (bx-ax)*(y-ay)/(by-ay)+ax:
            inside = not inside
    return inside
