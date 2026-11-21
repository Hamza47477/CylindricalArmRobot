# Builds / updates the robot inside FreeCAD (mm).  Two ways to use it:
#   * freecadcmd build_model.py        -> writes CylindricalArmRobot.FCStd
#   * server.py "mirror live into FreeCAD" -> the MCP RPC server runs this file and calls apply({"q": [...]})
import math
import FreeCAD as App, Part
from FreeCAD import Vector as V

NAME = "CylindricalArmRobot"
TOOL, SPH_R, SPH_C, COL_R, COL_H, ARM_R, L1, L2, LF = 300.0, 300.0, (600.0, 0.0, 650.0), 45.0, 1400.0, 28.0, 450.0, 450.0, 450.0


def _doc():
    try: return App.getDocument(NAME)
    except Exception: return App.newDocument(NAME)


def _col(o, c, t=0):
    if App.GuiUp:
        o.ViewObject.ShapeColor = tuple(x / 255 for x in c); o.ViewObject.Transparency = t


def _set(doc, name, shape, place, col, t=0):
    o = doc.getObject(name)
    if o is None:
        o = doc.addObject("Part::Feature", name); o.Label = name; _col(o, col, t)
    o.Shape = shape; o.Placement = place
    return o


def _mat(cx, cy, cz, p):   # placement from three column axes + origin
    return App.Placement(App.Matrix(cx[0], cy[0], cz[0], p[0], cx[1], cy[1], cz[1], p[1], cx[2], cy[2], cz[2], p[2], 0, 0, 0, 1))


def _cyl(r, a, b):
    d = V(*b) - V(*a)
    return Part.makeCylinder(r, d.Length, V(*a), d)


def _box(c, hx, hy, hz):   # local box centred at c
    s = Part.makeBox(2 * hx, 2 * hy, 2 * hz); s.translate(V(c[0] - hx, c[1] - hy, c[2] - hz)); return s


def _cr(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def apply(st):
    doc = _doc(); d2, t1, t2, t3, r1, r2 = st["q"]
    X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    ux = lambda a: (math.cos(a), math.sin(a), 0); gy = lambda a: (-math.sin(a), math.cos(a), 0)
    I = App.Placement()
    # FIXED: base plate + column (+ rail)
    _set(doc, "Base", Part.makeCylinder(200, 40), I, (78, 84, 98))
    col = Part.makeCylinder(COL_R, COL_H - 40, V(0, 0, 40)).fuse(_box((COL_R + 6, 0, (40 + COL_H) / 2), 8, 18, (COL_H - 40) / 2))
    _set(doc, "Column_fixed", col, I, (150, 158, 175))
    # d2: carriage (prismatic); th1: shoulder turret
    _set(doc, "Carriage_d2", _box((0, 0, -85), 80, 80, 40), _mat(X, Y, Z, (0, 0, d2)), (60, 130, 230))
    u1 = ux(t1); g1 = gy(t1); u12 = ux(t1 + t2); g12 = gy(t1 + t2); fd = ux(t1 + t2 + t3); gd = gy(t1 + t2 + t3)
    _set(doc, "Shoulder_th1", Part.makeCylinder(68, 90, V(0, 0, -45)), _mat(u1, g1, Z, (0, 0, d2)), (235, 140, 40))
    Jp = (L1 * u1[0], L1 * u1[1], d2); Ep = (Jp[0] + L2 * u12[0], Jp[1] + L2 * u12[1], d2); Wp = (Ep[0] + LF * fd[0], Ep[1] + LF * fd[1], d2)
    _set(doc, "Link1", _box((L1 / 2, 0, 0), L1 / 2, 30, 24), _mat(u1, g1, Z, (0, 0, d2)), (90, 120, 200))
    _set(doc, "Elbow1_th2", Part.makeCylinder(42, 90, V(0, 0, -45)), _mat(u1, g1, Z, Jp), (235, 140, 40))
    _set(doc, "Link2", _box((L2 / 2, 0, 0), L2 / 2, 28, 22), _mat(u12, g12, Z, Jp), (110, 150, 225))
    _set(doc, "Elbow2_th3", Part.makeCylinder(42, 90, V(0, 0, -45)), _mat(u12, g12, Z, Ep), (235, 140, 40))
    _set(doc, "Forearm", _box((LF / 2, 0, 0), LF / 2 - 55, 26, 20), _mat(fd, gd, Z, Ep), (170, 178, 195))
    W = Wp
    yp = tuple(math.cos(r1) * gd[i] + math.sin(r1) * Z[i] for i in range(3)); zp = tuple(-math.sin(r1) * gd[i] + math.cos(r1) * Z[i] for i in range(3))
    C = _mat(fd, yp, zp, W)
    _set(doc, "R1_roll_joint", _cyl(36, (-92, 0, 0), (-30, 0, 0)), C, (60, 190, 120))
    _set(doc, "R2_pitch_joint", _cyl(36, (0, -40, 0), (0, 40, 0)), C, (225, 85, 100))
    zt = tuple(math.cos(r2) * fd[i] - math.sin(r2) * zp[i] for i in range(3)); xt = _cr(yp, zt)
    T = _mat(xt, yp, zt, W)
    tool = Part.makeCylinder(14, 60).fuse(Part.makeCylinder(7, TOOL - 115, V(0, 0, 60))).fuse(_box((0, 0, TOOL - 30), 9, 34, 26)).fuse(_box((0, 0, TOOL - 8), 6, 40, 4))
    _set(doc, "Tool", tool, T, (240, 210, 80))
    # workspace sphere
    _set(doc, "Workspace_sphere", Part.makeSphere(SPH_R, V(*SPH_C)), I, (90, 150, 255), 85)
    tcp = tuple(W[i] + TOOL * zt[i] for i in range(3))
    _set(doc, "TCP", Part.makeSphere(8, V(*tcp)), I, (255, 210, 60))
    doc.recompute()
    if App.GuiUp:
        try:
            import FreeCADGui as Gui; Gui.updateGui()
        except Exception: pass
    return doc
