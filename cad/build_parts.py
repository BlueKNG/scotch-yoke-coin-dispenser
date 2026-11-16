"""Build every printable part of the coin elevator from geometry.json (CadQuery).

Run (from anywhere):
    uv run --python 3.12 --with cadquery --with shapely --with matplotlib python cad/build_parts.py

Writes cad/parts/<name>.step + .stl, cad/Mechanism_assembly.step (theta = 0),
cad/base_plate.dxf and cad/front_cover.dxf (laser-cut acrylic), and checks for
3D interference between all parts at 24 disc angles.

Frame: mm, origin at disc centre, x right, y up, z out of the wall (toward
the viewer). The base plate front face is z = 0. Layers:
    base -3..0 (3 mm acrylic) | disc 1..7 (groove 4..7) | hopper back 7.5..9 |
    slider + coin channel 9..13 | front cover 13..14.5 | rocker 15.5..19.5
Laser cut: base_plate, front_cover. Everything else is 3D printed. The axle
stub and pivot post are printed separately and bolted through the base (no
threads in plastic: through-bolts with nuts, a captive nut in the axle stub).
The disc rim is a spur gear driven by a pinion on a yellow TT gear motor (bought),
resting on a printed pad in front of the plate.
"""
import json, math, os, sys
import cadquery as cq
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely import affinity

HERE = os.path.dirname(os.path.abspath(__file__))
J = json.load(open(os.path.join(HERE, "geometry.json")))
p, R, H = J["p"], J["R"], J["hopper"]

# Layers [z0, z1] and print/assembly allowances (mm)
Z = dict(base=(-3, 0), disc=(1, 7), groove=(4, 7), back=(7.5, 9), chan=(9, 13),
         cover=(13, 14.5), rocker=(15.5, 19.5))
WALL = 2.5            # hopper wall thickness
FIT = 0.2             # sliding fit per side
AXLE_R = 5.0          # printed axle stub on the base (disc bore AXLE_R + FIT)
M3 = 1.6              # M3 clearance hole radius
POST_R = 3.5          # rocker pivot post radius
STANDOFF_R = 4.0
hw = p["chanW"] / 2


# ---------------------------------------------------------------- helpers
def prism(shape, z0, z1):
    """Extrude a shapely (Multi)Polygon between z0 and z1, holes included."""
    geoms = getattr(shape, "geoms", [shape])
    out = None
    for g in geoms:
        if g.is_empty:
            continue
        s = cq.Workplane("XY", origin=(0, 0, z0)).polyline(list(g.exterior.coords)[:-1]).close().extrude(z1 - z0)
        for ring in g.interiors:
            s = s.cut(cq.Workplane("XY", origin=(0, 0, z0 - 1)).polyline(list(ring.coords)[:-1]).close()
                      .extrude(z1 - z0 + 2))
        out = s if out is None else out.union(s)
    return out


def cyl(x, y, r, z0, z1):
    return cq.Workplane("XY", origin=(x, y, z0)).circle(r).extrude(z1 - z0)


def path_xy(key):
    a = H[key] if key in H else R[key]
    return list(zip(a[0], a[1]))


def gear2d(z, m, backlash=0.2, pa=math.radians(20)):
    """Involute spur gear outline (shapely), a tooth centred on +x. Each gear
    gives up half the total backlash. A radial flank runs below the base circle."""
    rp, ra, rf = m * z / 2, m * z / 2 + m, m * z / 2 - 1.25 * m
    rb = rp * math.cos(pa)
    s = math.pi * m / 2 - backlash / 2
    inv = lambda a: math.tan(a) - a
    half = lambda r: s / (2 * rp) + inv(pa) - inv(math.acos(min(1.0, rb / r)))
    rs = [max(rb, rf) + (ra - max(rb, rf)) * k / 12 for k in range(13)]
    fl = [(r, half(r)) for r in rs]
    fl = [(rf - 0.3, half(max(rb, rf)))] + fl
    teeth = []
    for k in range(z):
        c = 2 * math.pi * k / z
        pts = [(r * math.cos(c - a), r * math.sin(c - a)) for r, a in fl]
        pts += [(r * math.cos(c + a), r * math.sin(c + a)) for r, a in reversed(fl)]
        teeth.append(Polygon(pts))
    return unary_union([Point(0, 0).buffer(rf, 128)] + teeth)


# ---------------------------------------------------------------- 2D layout
loop = path_xy("path")[:-1]                                    # closed coin centre path
chan = LineString(loop + [loop[0]]).buffer(hw, join_style=2, mitre_limit=5)
col_ext = box(-hw, H["yBot"] - 1, hw, H["yC0"])                # column below the feed: stem enters here
outline = unary_union([Polygon(loop).buffer(hw + WALL, join_style=2, mitre_limit=5),
                       box(-hw - WALL, H["yBot"], hw + WALL, H["yC0"])])
outline = outline.intersection(box(-200, H["yBot"], 200, 400))  # mitre tip at the feed corner pokes below
win = box(hw - 0.5, H["window"][0], hw + WALL + 1, H["window"][1])
walls2d = outline.difference(unary_union([chan, col_ext, win]))

# Hopper standoffs: in solid wall area, outside the disc and the rocker sweep
cand = []
safe = walls2d.buffer(-(STANDOFF_R + 0.5))
for x in range(-90, 30, 2):
    for y in range(50, 210, 2):
        q = Point(x, y)
        if safe.contains(q) and math.hypot(x, y) > p["Rd"] + STANDOFF_R + 2:
            cand.append((x, y))
standoffs = [max(cand, key=lambda c: c[1])]                    # farthest-point pick of 4
while len(standoffs) < 4:
    standoffs.append(max(cand, key=lambda c: min(math.dist(c, s) for s in standoffs)))

# Pawl pin sweep (for the cover slot)
pin_path = LineString(path_xy("G")).buffer(p["rGate"] + 0.6)

# ---------------------------------------------------------------- parts
parts = {}

# Base plate (A4, 3 mm acrylic, laser cut): only through holes
M5 = 2.75
base = cq.Workplane("XY", origin=(0, 0, Z["base"][0])).center(23, 56.5).rect(210, 297).extrude(Z["base"][1] - Z["base"][0])
for x, y, r in ([(0, 0, M5), (p["a"], p["h"], M3)] + [(x, y, M3) for x, y in standoffs]
                + [(x, y, 2.5) for x, y in [(-72, -82), (118, -82), (-72, 195), (118, 195)]]):
    base = base.cut(cyl(x, y, r, -5, 1))
parts["base_plate"] = base

# Axle stub (printed): disc turns on it. No threads in plastic: M3 x 10 + washer from
# behind the plate into a captive M3 nut sunk in the stub top (nothing may stick out
# above z = 7: the slider and hopper back pass over the disc centre). A spigot fills
# the 5.5 mm base hole (an M5 clearance hole) so the stub stays centred.
NUT_AF = 5.5 + 0.3                                              # M3 nut across flats + fit
nut_pocket = cq.Workplane("XY", origin=(0, 0, 4.0)).polygon(6, NUT_AF / math.cos(math.pi / 6)).extrude(4)
stub = cyl(0, 0, AXLE_R, 0, Z["disc"][1]).union(cyl(0, 0, M5 - 0.1, -2.5, 0))
parts["axle_stub"] = stub.cut(cyl(0, 0, M3, -5, 10)).cut(nut_pocket)

# Rocker pivot post (printed): one M3 x 30 through the base, the post and the rocker,
# washer + nylock nut on top. The bolt shank is the pivot pin; no threads in plastic.
post = cyl(p["a"], p["h"], POST_R, 0, Z["rocker"][0])
parts["pivot_post"] = post.cut(cyl(p["a"], p["h"], M3, -1, 30))

# Disc: crank + face cam groove + crank pin, gear teeth on the rim
halfW = p["rr"] + p["clear"]
groove = LineString(list(zip(R["C"][0], R["C"][1]))).buffer(halfW, join_style=1)
gear_disc = gear2d(p["Ng"], p["mGear"])
disc = prism(gear_disc, *Z["disc"]).cut(cyl(0, 0, AXLE_R + FIT, 0, 8))
disc = disc.cut(prism(groove, Z["groove"][0], Z["groove"][1] + 1))
disc = disc.union(cyl(p["e"], 0, p["rPin"], Z["disc"][1], Z["chan"][1] - 0.4))
parts["disc"] = disc

# Motor pinion (printed) on the yellow TT gear motor, single output shaft. The motor lies flat in front of
# the plate with the shaft face toward it; the shaft (9.1 long) passes through the pinion.
GB_FACE = 11.0                                                  # gearbox shaft face z (shaft tip at z = 1.9)
cdist = p["mGear"] * (p["Ng"] + p["Np"]) / 2
Q = (cdist * math.cos(p["gearAng"]), cdist * math.sin(p["gearAng"]))
gear_pin = gear2d(p["Np"], p["mGear"])
# Double-D bore. Drawing: round 5.5, flats 3.7 (user caliper read about 4.0). Printed holes
# shrink, so add a fit; print cad/parts/tt_shaft_fit_test.stl to confirm it and set TT_FLAT.
TT_ROUND, TT_FLAT = 5.7, 3.9                                    # printed bore: round dia, flat-to-flat (mm)
def tt_bore(flat, rnd=TT_ROUND):
    return Point(0, 0).buffer(rnd / 2, 64).intersection(box(-rnd, -flat / 2, rnd, flat / 2))
bore = tt_bore(TT_FLAT)
pinion = prism(gear_pin.difference(bore), *Z["disc"])
pinion = pinion.union(prism(Point(0, 0).buffer(5.5, 64).difference(bore), Z["disc"][1], GB_FACE - 0.5))
# Shaft fit test strip: 6 bores, flats 3.7 .. 4.2 from the notched corner
FIT_FLATS = [3.7, 3.8, 3.9, 4.0, 4.1, 4.2]
fit_test = prism(box(0, 0, 10 * len(FIT_FLATS), 10), 0, 6)
for k, f in enumerate(FIT_FLATS):
    fit_test = fit_test.cut(prism(affinity.translate(tt_bore(f), 5 + 10 * k, 5), -1, 7))
fit_test = fit_test.cut(prism(Polygon([(0, 0), (3, 0), (0, 3)]), -1, 7))
parts["tt_shaft_fit_test"] = fit_test
parts["pinion"] = pinion                                        # modelled at its own centre

# Pinion phase that meshes with the disc at theta = 0 (least 2D overlap)
gear_pin_rpa = 2 * math.pi / p["Np"]
def pin_at(psi):
    from shapely import affinity
    return affinity.translate(affinity.rotate(gear_pin, psi, origin=(0, 0), use_radians=True), *Q)
PSI0 = min((gear_pin_rpa * k / 200 for k in range(200)), key=lambda a: pin_at(a).intersection(gear_disc).area)

# TT motor mount (printed pad): the gearbox rests on a pad (z 0..GB_FACE) under its two
# 2.8 mm through holes; two M2.5 x 35 bolts go gearbox -> pad -> base plate -> nut.
# Motor-local frame: d along the body (shaft at d = 0, gearbox end d = -11, gearbox/motor
# joint d = +25, motor end d = +53.4), w across (gearbox 22.3 wide). The body points along
# the tangent (+30 deg) so it stays on the plate and clear of the rocker.
TT_DIR = math.radians(30)
dd = (math.cos(TT_DIR), math.sin(TT_DIR))
ww = (-dd[1], dd[0])
def local(pd, pw):
    return (Q[0] + pd * dd[0] + pw * ww[0], Q[1] + pd * dd[1] + pw * ww[1])
def lbox(d0, d1, w0, w1):
    return Polygon([local(d0, w0), local(d1, w0), local(d1, w1), local(d0, w1)])
TT_HOLES = [local(20.1, s * 8.75) for s in (-1, 1)]             # 2 x dia 2.8 through the gearbox
M25 = 1.45                                                      # M2.5 clearance radius
pad2d = (lbox(13.0, 25.0, -11.15, 11.15)
         .difference(Point(*Q).buffer(18.0, 64))                # pinion (tip r 16.5) and hub
         .difference(Point(0, 0).buffer(p["Rd"] + 1.5, 128)))  # disc rim
brk = prism(pad2d, 0, GB_FACE)
for x, y in TT_HOLES:
    brk = brk.cut(cyl(x, y, M25, -1, GB_FACE + 1))
parts["motor_bracket"] = brk
uu = (math.cos(p["gearAng"]), math.sin(p["gearAng"]))           # radial from the disc centre
for x, y in TT_HOLES:                                           # base: radial slots +/-1 mm to set the gear mesh
    slot = LineString([(x - uu[0], y - uu[1]), (x + uu[0], y + uu[1])]).buffer(M25, 32)
    parts["base_plate"] = parts["base_plate"].cut(prism(slot, -5, 1))
bracket_holes = TT_HOLES
# TT envelope for the assembly and the clash check only (bought part): gearbox + motor can
# (its end clip is up to 2 mm proud of the gearbox faces) + output shaft.
motor = prism(lbox(-11.0, 25.0, -11.15, 11.15), GB_FACE, GB_FACE + 18.7).union(
    prism(lbox(25.0, 53.4, -8.75, 8.75), GB_FACE - 2.0, GB_FACE + 18.7 + 2.0))

# Slider: yoke with horizontal slot + stem (coin pusher)
sw = p["rPin"] + FIT                                            # slot half-width
yoke2d = box(-(p["e"] + sw + 4), -6, p["e"] + sw + 4, 6).difference(
    LineString([(-p["e"], 0), (p["e"], 0)]).buffer(sw))
stem2d = box(-p["stemW"] / 2, 5, p["stemW"] / 2, p["stemLen"])
parts["slider"] = prism(unary_union([yoke2d, stem2d]), *Z["chan"])

# Hopper: back sheet + walls, standoffs with M3 holes
hop = prism(outline, *Z["back"]).union(prism(walls2d, *Z["chan"]))
for x, y in standoffs:
    hop = hop.union(cyl(x, y, STANDOFF_R, 0, Z["back"][0]))
    hop = hop.cut(cyl(x, y, M3, -1, Z["chan"][1] + 1))        # M3 x 20 through-bolt + nut
parts["hopper"] = hop

# Front cover (clear acrylic or PETG): coin retainer, pawl slot, slider guide strip + rails
g0, g1 = p["guideY"]
strip = box(-hw - WALL, g0 - 2, hw + WALL, H["yBot"] + 1)
cover2d = unary_union([outline, strip]).difference(pin_path)
for x, y in standoffs:
    cover2d = cover2d.difference(Point(x, y).buffer(M3))
cover = prism(cover2d, *Z["cover"])
rx0, rx1 = p["stemW"] / 2 + FIT, p["stemW"] / 2 + FIT + 3
for s in (-1, 1):
    cover = cover.union(prism(box(min(s * rx0, s * rx1), g0, max(s * rx0, s * rx1), g1), Z["chan"][0] + 0.3,
                              Z["cover"][0]))
parts["front_cover"] = cover

# Rocker (pawl), modelled at phi = 0 with the pivot at the origin
L1, L2, b = p["L1"], p["L2"], p["b"]
arms = unary_union([LineString([(0, -L2), (0, L1), (-b, L1)]).buffer(3, join_style=1),
                    Point(0, 0).buffer(5.5), Point(0, -L2).buffer(4), Point(-b, L1).buffer(3.5)])
rk = prism(arms.difference(Point(0, 0).buffer(M3)), *Z["rocker"])
rk = rk.union(cyl(0, -L2, p["rr"], Z["groove"][0] + 0.5, Z["rocker"][0]))        # follower pin
rk = rk.union(cyl(-b, L1, p["rGate"], Z["chan"][0] + 0.8, Z["rocker"][0]))       # pawl pin
parts["rocker"] = rk

# Coin token (print 10, or use real coins of the same diameter)
parts["coin_token"] = cyl(0, 0, p["Dcoin"] / 2, 0, 2.5)


# ---------------------------------------------------------------- placement
def interp(arr, th):
    ths = R["th"]
    th = th % (2 * math.pi)
    i = min(range(len(ths) - 1), key=lambda k: abs(ths[k] - th))
    return arr[i]


def placed(th):
    """Parts in assembly position at disc angle th [rad]."""
    yS, phi = interp(R["yS"], th), interp(R["phi"], th)
    return {
        "base_plate": parts["base_plate"],
        "axle_stub": parts["axle_stub"],
        "pivot_post": parts["pivot_post"],
        "disc": parts["disc"].rotate((0, 0, 0), (0, 0, 1), math.degrees(th)),
        "slider": parts["slider"].translate((0, yS, 0)),
        "hopper": parts["hopper"],
        "front_cover": parts["front_cover"],
        "rocker": parts["rocker"].rotate((0, 0, 0), (0, 0, 1), math.degrees(phi)).translate((p["a"], p["h"], 0)),
        "pinion": parts["pinion"].rotate((0, 0, 0), (0, 0, 1), math.degrees(PSI0 - th * p["Ng"] / p["Np"]))
                                 .translate((Q[0], Q[1], 0)),
        "motor_bracket": parts["motor_bracket"],
        "motor_TT": motor,
    }


def overlap(a, b):
    ba, bb = a.val().BoundingBox(), b.val().BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
            or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
        return 0.0
    i = a.intersect(b)
    return sum(s.Volume() for s in i.solids().vals()) if i.solids().size() else 0.0


# ---------------------------------------------------------------- outputs
out = os.path.join(HERE, "parts")
os.makedirs(out, exist_ok=True)
for name, s in parts.items():
    cq.exporters.export(s, os.path.join(out, name + ".step"))
    cq.exporters.export(s, os.path.join(out, name + ".stl"), tolerance=0.05, angularTolerance=0.1)
    bb = s.val().BoundingBox()
    print(f"{name:12s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f} mm  vol {s.val().Volume()/1000:7.2f} cm^3")
def slice_at(s, z):
    """Section of a part at absolute height z (section() is relative to the workplane origin)."""
    return cq.Workplane("XY").add(s.val()).section(z)


# Bought motor envelope (visual only) and pinion phase for the Simscape model
cq.exporters.export(motor, os.path.join(out, "motor_TT.step"))
json.dump(dict(psi0=PSI0, Q=Q), open(os.path.join(out, "pinion_phase.json"), "w"))

cq.exporters.export(slice_at(parts["front_cover"], Z["cover"][0] + 0.5), os.path.join(HERE, "front_cover.dxf"))
cq.exporters.export(slice_at(parts["base_plate"], -1.5), os.path.join(HERE, "base_plate.dxf"))

rgb = lambda r, g, b: (r / 255, g / 255, b / 255)     # as-built colours (fabrication/COLOURS.md)
colors = dict(base_plate=rgb(20, 40, 120), axle_stub=rgb(25, 25, 25), pivot_post=rgb(25, 25, 25), disc=rgb(110, 180, 235),
              slider=rgb(200, 30, 30), hopper=rgb(70, 70, 75), front_cover=(0.7, 0.9, 1.0, 0.35),
              rocker=rgb(240, 200, 20), pinion=rgb(0, 150, 90), motor_bracket=rgb(25, 25, 25), motor_TT=rgb(240, 200, 20))
asm = cq.Assembly()
for name, s in placed(0.0).items():
    asm.add(s, name=name, color=cq.Color(*colors[name]))
asm.save(os.path.join(HERE, "Mechanism_assembly.step"))

# Interference check over one turn
worst = {}
for k in range(24):
    th = 2 * math.pi * k / 24
    P = placed(th)
    names = list(P)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            v = overlap(P[names[i]], P[names[j]])
            key = names[i] + " / " + names[j]
            if v > worst.get(key, (0, 0))[0]:
                worst[key] = (v, math.degrees(th))
bad = {k: v for k, v in worst.items() if v[0] > 1e-3}
print("Interference check (24 angles):", "OK, no overlaps" if not bad else "")
for k, (v, d) in bad.items():
    print(f"  CLASH {k}: {v:.3f} mm^3 at theta = {d:.0f} deg")
print("Standoffs at", [(round(x), round(y)) for x, y in standoffs])
print(f"Pinion centre ({Q[0]:.2f}, {Q[1]:.2f}) mm, phase {math.degrees(PSI0):.2f} deg, "
      f"2D tooth overlap {pin_at(PSI0).intersection(gear_disc).area:.4f} mm^2")
print("Base plate TT motor holes (M2.5 clearance, cut):",
      [(round(x, 1), round(y, 1)) for x, y in bracket_holes])
sys.exit(1 if bad else 0)
