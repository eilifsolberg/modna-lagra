"""
Rigid chain for the fork - parametric, 3D-printable (build123d)
===============================================================

One printed chain with steel pins that pushes the fork into a row and holds
it out there with a fruit on it. It is rigid one way only: the pins run along
its top and the links butt against each other with flat faces underneath, so
a downward load (the fruit, always) closes the joints and the chain stays
straight, while it can still curl upwards. Behind the cradle it curls a
quarter turn up round the drive sprocket into a straight vertical magazine,
so a short carriage reaches the front of a long row.

  - Strength: steel pins in double shear, PLA links loaded in their plane.
  - Stiffness: under load the butt faces are pressed together, so the only
    play is the pin clearance in the tongues - gravity always takes it up the
    same way, so the droop is repeatable and the elevator's Z axis corrects it
    (a droop table per fork extension). Keep the clearance small (reamed
    holes) and there's little to correct.
  - Drive: the pin ends stick out at the sides and run in a pair of sprocket
    plates above the chain; the sprocket also holds the chain down against
    the lever of the fruit.
  - Keel: a narrow fin under each link. The chain body rides above the
    shelf's rods, the keel runs in the gap between the two rods the fruit
    rests on, and the carriage supports the chain through it - low enough
    that a fruit on the tines passes over that support.

Link frame (mm): v along the chain (pin A at v = 0, pin B at v = PITCH),
u across it, w up from the bottom face (the butt side). Each link has a tongue
in the middle layer round pin B and a clevis (two outer plates) round pin A,
so every link is the same part. Pins are pressed into the clevis and turn in
the tongue. The keel hangs below the bottom face, in the middle of the tongue.

Run:    python rigid_chain.py
Output: STL files for printing in ./rigid_chain_print/, a STEP of the bench
test assembly, a joint check and a load / droop report.
"""

import math
from pathlib import Path

from build123d import (
    Align,
    Circle,
    Color,
    Compound,
    Polygon,
    Pos,
    Rectangle,
    Rot,
    export_step,
    export_stl,
    extrude,
)

# ---------------------------------------------------------------
# PARAMETERS
# ---------------------------------------------------------------
PITCH = 12  # pin to pin
CHAIN_W = 16  # width of the chain body
CHAIN_H = 31  # height; stiffness grows with its cube
TONGUE_W = 7  # middle layer (tongue); the two outer layers are the clevis
KEEL_W = 4  # keel under the tongue: width (fits a rod gap with clearance)
KEEL_H = 7  # ... depth below the bottom face
KEEL_END = 1  # ... stops this short of each link end, so keels never touch
KEEL_CHAMFER = 1.5  # ... chamfered ends, to ride onto the carriage's support

# Pins: 3 mm steel. Silver steel (h8) cut to length; ream the tongue holes
# with a 3 mm H7 reamer (turning fit), the clevis holes are printed tight and
# the pin is pressed in.
PIN_D = 3.0
PIN_E = 4.5  # pin centre to the chain's top face (= knuckle radius)
PIN_STICK = 4  # pin ends stick out this far each side, for the sprocket
CLEVIS_HOLE = 2.9  # printed hole, pin pressed in
TONGUE_HOLE = 3.1  # printed hole, reamed to 3.0 H7
PIN_CLEAR = 0.02  # diametral pin clearance after reaming (for the droop estimate)

FIT = 0.35  # radial clearance knuckle <-> the neighbour's pocket
FIT_U = 0.3  # side clearance tongue <-> clevis

# Drive: a sprocket pair on the pin ends, above the chain where it turns up
SPROCKET_N = 10  # teeth; also how tight the chain turns (360 / N per link)
SPROCKET_BORE = 5  # shaft
SPROCKET_T = 4  # plate thickness (one plate each side of the chain)
MAG_FIT = 1.0  # clearance chain <-> magazine channel, front and back

# Material and structure. ETA knocks the solid-section stiffness down for the
# joints - measure it with the bench test and put the result here.
MATERIALS = {  # E (MPa), tensile (MPa), bearing (MPa), density (g/cm3)
    "PLA": (3000, 45, 60, 1.24),
    "PETG": (1900, 40, 50, 1.27),
}
MATERIAL = "PLA"  # stiffer; fine at the machine's 7-26 degC
INFILL = 0.8  # solid fraction of a printed link (for its weight)
ETA = 0.5  # effective / solid bending stiffness of the chain
PIN_SHEAR = 300  # allowable shear stress in the steel pins (MPa)
BUTT_CONTACT = 3  # height of the butt faces' contact band at the bottom (mm)
DYN = 1.5  # dynamic factor on loads for strength (elevator jerks)
DROOP_MAX = 3.0  # elastic droop at the fruit that is acceptable without
# knowing the fruit's weight (mm)

# Fruit: name -> (min mass g, max mass g, max length mm along the fork)
FRUIT = {
    "Avocado (Hass)": (150, 350, 115),
    "Mango (Ataulfo)": (200, 350, 130),
    "Mango (Kent / Keitt)": (500, 1000, 150),
}

# Stand-alone run (the machine model passes its own numbers)
REACH_FREE = 228  # fork reach beyond the carriage's front support (mm)
BACK_SPAN = 150  # front support -> sprocket hold-down (mm)
HEAD_G = 40  # fork head (pusher + tines) mass, g
PUSHER_T = 6
BENCH_LINKS = 22  # links in the bench test bar
CLAMP_LINKS = 3  # links held in the bench clamp

# ---------------------------------------------------------------
# DERIVED QUANTITIES
# ---------------------------------------------------------------
t_clevis = (CHAIN_W - TONGUE_W - 2 * FIT_U) / 2  # outer plate thickness
pin_len = CHAIN_W + 2 * PIN_STICK
w_pin = CHAIN_H - PIN_E  # pin line above the bottom face
chain_depth = CHAIN_H + KEEL_H  # keel bottom to top face
lever = w_pin - BUTT_CONTACT / 2  # pins (tension) <-> butt faces (compression)
butt_h = CHAIN_H - 2 * PIN_E - FIT  # height of the flat butt faces
bend_step = 2 * math.pi / SPROCKET_N  # angle between links on the sprocket
r_sprocket = PITCH / (2 * math.sin(bend_step / 2))  # pitch radius
r_plate = r_sprocket + PIN_D / 2 + 2  # sprocket plate outer radius
# Tightest curl the links allow before their top edges touch
x_free = math.sqrt((PIN_E + FIT) ** 2 - PIN_E**2)
bend_max = 2 * math.atan2(x_free, PIN_E)

E_MOD, S_TENS, S_BEAR, RHO = MATERIALS[MATERIAL]
EI = ETA * E_MOD * CHAIN_W * CHAIN_H**3 / 12  # N mm2
keel_area = KEEL_W * KEEL_H * (PITCH - 2 * KEEL_END - KEEL_CHAMFER) / PITCH
q_chain = INFILL * RHO * 1e-3 * (CHAIN_W * CHAIN_H + keel_area) * 9.81e-3  # N/mm
assert KEEL_W <= TONGUE_W, "the keel must sit under the tongue"


# ---------------------------------------------------------------
# LINK GEOMETRY
# ---------------------------------------------------------------
def _rect(x0, y0, x1, y1):
    return Pos(x0, y0) * Rectangle(x1 - x0, y1 - y0, align=(Align.MIN, Align.MIN))


def _circle(x, y, r):
    return Pos(x, y) * Circle(r)


def link_profile(layer):
    """Side profile (v, w) of one layer of a link. layer: "tongue", "keel"
    (the middle of the tongue, with the keel below it), "clevis", or "gap"
    (the thin slice between them)."""
    p = PITCH
    prof = _rect(0, 0, p, CHAIN_H)
    if layer == "keel":
        e, c = KEEL_END, KEEL_CHAMFER
        prof += Polygon(  # counter-clockwise, so the face's normal stays +Z
            (e, 0.1), (e + c, -KEEL_H), (p - e - c, -KEEL_H), (p - e, 0.1), align=None
        )
    if layer in ("tongue", "keel"):
        prof += _circle(p, w_pin, PIN_E)
        prof -= _circle(0, w_pin, PIN_E + FIT)
        prof -= _circle(p, w_pin, TONGUE_HOLE / 2)
    elif layer == "clevis":
        prof += _circle(0, w_pin, PIN_E)
        prof -= _circle(p, w_pin, PIN_E + FIT)
        prof -= _circle(0, w_pin, CLEVIS_HOLE / 2)
    else:
        prof -= _circle(0, w_pin, PIN_E + FIT)
        prof -= _circle(p, w_pin, PIN_E + FIT)
    return prof


LAYERS = [  # (layer, u0, u1)
    ("clevis", -CHAIN_W / 2, -TONGUE_W / 2 - FIT_U),
    ("gap", -TONGUE_W / 2 - FIT_U, -TONGUE_W / 2),
    ("tongue", -TONGUE_W / 2, -KEEL_W / 2),
    ("keel", -KEEL_W / 2, KEEL_W / 2),
    ("tongue", KEEL_W / 2, TONGUE_W / 2),
    ("gap", TONGUE_W / 2, TONGUE_W / 2 + FIT_U),
    ("clevis", TONGUE_W / 2 + FIT_U, CHAIN_W / 2),
]


def link_solid():
    """One link in the link frame: X = v, Y = u, Z = w."""
    solid = None
    for layer, u0, u1 in LAYERS:
        slab = Pos(0, 0, u0) * extrude(link_profile(layer), amount=u1 - u0)
        solid = slab if solid is None else solid + slab
    return Rot(90, 0, 0) * solid  # profile (v, w) -> (X, Z)


def pin_solid():
    """Steel pin along Y, centred at the origin."""
    return Rot(90, 0, 0) * extrude(Circle(PIN_D / 2), amount=pin_len / 2, both=True)


def straight_chain(n, v0=0.0, link=None):
    """n links in a straight line, first pin at v = v0, bottom face at w = 0
    (the keels hang below it).
    Returns [(shape, label, colour)] in the chain frame (X = v, Z = w)."""
    link = link or link_solid()
    pin = pin_solid()
    out = []
    for k in range(n):
        v = v0 + k * PITCH
        out.append((Pos(v, 0, 0) * link, "Rigid chain – link", "#cfd8dc"))
        out.append((Pos(v, 0, w_pin) * pin, "Rigid chain – pin", "#90a4ae"))
    return out


def sprocket_half():
    """One side plate of the drive sprocket, with half the hub. The pin ends
    sit in the tooth gaps; two halves bolt together around the chain."""
    r_root = r_sprocket - PIN_D / 2 - 0.3
    plate = Circle(r_plate)
    for i in range(SPROCKET_N):
        a = i * bend_step
        c = (r_sprocket * math.cos(a), r_sprocket * math.sin(a))
        plate -= Pos(*c) * Circle(PIN_D / 2 + 0.3)
        # open the gap outwards so the pins can enter and leave
        plate -= (
            Pos(*c)
            * Rot(0, 0, math.degrees(a))
            * Rectangle(10, PIN_D + 0.6, align=(Align.MIN, Align.CENTER))
        )
    plate = extrude(plate - Circle(SPROCKET_BORE / 2), amount=SPROCKET_T)
    # hub reaches half way across the chain, inside the curl of the links
    r_hub = r_sprocket - PIN_E - 1.5
    hub_len = CHAIN_W / 2 + FIT_U + 0.5
    hub = Pos(0, 0, SPROCKET_T) * extrude(
        Circle(r_hub) - Circle(SPROCKET_BORE / 2), amount=hub_len
    )
    part = plate + hub
    for a in (0, math.pi):  # bolts joining the two halves
        hole = Pos(r_hub * 0.6 * math.cos(a), r_hub * 0.6 * math.sin(a)) * Circle(1.6)
        part -= extrude(hole, amount=SPROCKET_T + hub_len)
    assert r_root > r_hub > SPROCKET_BORE / 2 + 2, "sprocket hub doesn't fit"
    return part


def bench_clamp():
    """U-channel that holds the root of a chain for the bench test. The chain
    stands on its keels on the floor, the pin ends drop into slots in the side
    walls; bolt it to a table edge."""
    length = CLAMP_LINKS * PITCH + PITCH / 2
    inner = pin_len + 0.6
    wall, floor = 5, 5
    h = chain_depth + 4
    w_slot = floor + KEEL_H + w_pin  # pin line above the clamp's underside
    part = extrude(_rect(0, 0, length, floor + h), amount=inner / 2 + wall, both=True)
    part -= Pos(0, 0, floor) * extrude(
        _rect(0, 0, length, h + 1), amount=CHAIN_W / 2 + 0.3, both=True
    )
    for k in range(1, CLAMP_LINKS + 1):  # slots for the pin ends, from the top
        v = k * PITCH
        slot = _rect(v - PIN_D / 2 - 0.15, w_slot, v + PIN_D / 2 + 0.15, floor + h + 1)
        slot += _circle(v, w_slot, PIN_D / 2 + 0.15)
        part -= extrude(slot, amount=inner / 2, both=True)
    part = Rot(90, 0, 0) * part  # (v, w) -> (X, Z)
    for x in (length * 0.25, length * 0.75):  # mounting holes through the floor
        part -= Pos(x, 0, 0) * extrude(Circle(2.2), amount=floor)
    return part


# ---------------------------------------------------------------
# JOINT CHECK
# ---------------------------------------------------------------
def joint_overlap(angle_deg):
    """Largest overlap (mm2) between two neighbouring links, the second turned
    angle_deg about their shared pin (positive = curling up). Should be 0 up
    to the sprocket's angle, and > 0 for any downward bend (the butt faces
    stop it)."""
    worst = 0.0
    for layer in ("tongue", "keel", "clevis", "gap"):
        a = link_profile(layer)
        b = Pos(PITCH, w_pin) * Rot(0, 0, angle_deg) * Pos(0, -w_pin) * a
        inter = a & b
        worst = max(worst, inter.area if inter is not None else 0.0)
    return worst


# ---------------------------------------------------------------
# LOADS AND DROOP
# ---------------------------------------------------------------
def fork_load(
    mass_g, fruit_len, reach_free, back_span, head_g=HEAD_G, pusher_t=PUSHER_T
):
    """The chain sticks out `reach_free` beyond the carriage's front support
    and is held down by the sprocket `back_span` behind it. The fruit rests on
    the tines with its rear against the pusher face.
    Returns a dict of loads, safety factor and droop at the fruit's centre."""
    L, Lb = reach_free, back_span
    e = pusher_t + fruit_len / 2  # chain end -> fruit centre
    P, Ph, q = mass_g * 9.81e-3, head_g * 9.81e-3, q_chain
    # cantilever from the front support: end load, end moment, own weight
    d_end = P * L**3 / (3 * EI) + P * e * L**2 / (2 * EI) + Ph * L**3 / (3 * EI)
    d_end += q * L**4 / (8 * EI)
    th_end = P * L**2 / (2 * EI) + P * e * L / EI + Ph * L**2 / (2 * EI)
    th_end += q * L**3 / (6 * EI)
    m_root = P * (L + e) + Ph * L + q * L**2 / 2
    th_root = m_root * Lb / (3 * EI)  # the back span bends too
    droop = d_end + th_end * e + th_root * (L + e)
    # pin clearance: each tongue slides PIN_CLEAR / 2 along its pin while the
    # butt faces stay closed - a steady curvature, always the same way
    kappa = PIN_CLEAR / 2 / (PITCH * lever)
    play = kappa * (L**2 / 2 + L * e) + kappa * Lb / 2 * (L + e)
    tilt = kappa * (L + Lb / 2) + th_end + th_root  # tine slope (rad)
    # strength, with the dynamic factor
    f = DYN * m_root / lever  # pin tension = butt face compression (N)
    sf = min(
        PIN_SHEAR / (f / (2 * math.pi * PIN_D**2 / 4)),  # pin, double shear
        S_BEAR / (f / (PIN_D * TONGUE_W)),  # tongue bearing
        S_BEAR / (f / (PIN_D * 2 * t_clevis)),  # clevis bearing
        S_TENS / (f / (2 * (PIN_E - TONGUE_HOLE / 2) * TONGUE_W)),  # tongue ring
        S_BEAR / (f / (BUTT_CONTACT * CHAIN_W)),  # butt faces
    )
    return {
        "moment": m_root / 1000,  # N m, static
        "force": f,
        "sf": sf,
        "droop": droop,
        "play": play,
        "tilt": math.degrees(tilt),
    }


def max_mass(fruit_len, reach_free, back_span, droop_max=DROOP_MAX):
    """Largest fruit (g) that droops at most droop_max (mm)."""
    base = fork_load(0, fruit_len, reach_free, back_span)["droop"]
    per_g = fork_load(1000, fruit_len, reach_free, back_span)["droop"] - base
    return max(0.0, (droop_max - base) / per_g * 1000)


def report_lines(
    reach_free, back_span, fruit_lens=None, head_g=HEAD_G, pusher_t=PUSHER_T
):
    """Load table for the report. fruit_lens overrides FRUIT lengths by name."""
    lines = [
        (
            f" Chain: {MATERIAL}, pitch {PITCH}, {CHAIN_W} x {CHAIN_H} mm"
            f" + keel {KEEL_W} x {KEEL_H}, pins {PIN_D} x {pin_len:.0f} mm steel"
        ),
        f" Reach beyond support:      {reach_free:.0f} mm  (back span {back_span:.0f} mm)",
        "   fruit                   mass    moment  SF   droop  +play  tilt",
    ]
    for name, (m0, m1, flen) in FRUIT.items():
        flen = (fruit_lens or {}).get(name, flen)
        for m in (m0, m1):
            r = fork_load(m, flen, reach_free, back_span, head_g, pusher_t)
            flag = "  !" if r["sf"] < 3 or r["droop"] > DROOP_MAX else ""
            lines.append(
                f"   {name:<22}{m:5.0f} g {r['moment']:5.2f} Nm {r['sf']:4.1f}"
                f" {r['droop']:5.1f}  {r['play']:5.1f} {r['tilt']:5.2f}°{flag}"
            )
    lines.append(
        "   (droop mm at the fruit: elastic, then pin play; both are corrected by"
    )
    lines.append(
        f"    the elevator Z. SF on {DYN} x load. Max fruit for ≤ {DROOP_MAX:.0f} mm"
        f" elastic droop: {max_mass(130, reach_free, back_span):.0f} g)"
    )
    return lines


# ---------------------------------------------------------------
# STAND-ALONE: print parts, bench test and report
# ---------------------------------------------------------------
if __name__ == "__main__":
    out = Path("rigid_chain_print")
    out.mkdir(exist_ok=True)
    link = link_solid()
    # print standing on the keel, with supports under the bottom (butt) face
    # either side of it (PrusaSlicer: supports on build plate only); pin holes
    # horizontal
    export_stl(link, str(out / "link.stl"))
    export_stl(sprocket_half(), str(out / "sprocket_half.stl"))
    export_stl(bench_clamp(), str(out / "bench_clamp.stl"))

    # bench test: clamp + a chain as long as the real reach
    parts = []
    clamp = bench_clamp()
    clamp.label, clamp.color = "Bench clamp", Color("#ffb300")
    parts.append(clamp)
    for shp, lab, col in straight_chain(BENCH_LINKS, link=link):
        shp = Pos(0, 0, 5 + KEEL_H) * shp  # keels on the clamp floor
        shp.label, shp.color = lab, Color(col)
        parts.append(shp)
    export_step(
        Compound(children=parts, label="Rigid chain bench test"),
        str(out / "bench_test.step"),
    )

    print("=" * 64)
    print(" RIGID CHAIN")
    print("=" * 64)
    print(
        f" Link: {PITCH} x {CHAIN_W} x {CHAIN_H} mm, clevis plates {t_clevis:.1f} mm,"
        f" tongue {TONGUE_W} mm, butt faces {butt_h:.0f} mm high,"
        f" keel {KEEL_W} x {KEEL_H} mm"
    )
    print(
        f" Curl: sprocket {SPROCKET_N} teeth, pitch Ø {2 * r_sprocket:.1f} mm,"
        f" {math.degrees(bend_step):.0f}° per link (links allow"
        f" {math.degrees(bend_max):.0f}°)"
    )
    up = joint_overlap(math.degrees(bend_step))
    down = joint_overlap(-1)
    print(
        f" Joint check: overlap {up:.3f} mm² curled up on the sprocket"
        + ("  (free)" if up < 0.01 else "  ! links collide")
        + f", {down:.1f} mm² bent 1° down"
        + ("  (butt faces stop it)" if down > 0.1 else "  ! not rigid downwards")
    )
    print(
        f" Stiffness EI:   {EI / 1e6:.1f} N m² (ETA {ETA}),"
        f" chain {q_chain * 1000 / 9.81:.2f} g/mm"
    )
    for line in report_lines(REACH_FREE, BACK_SPAN):
        print(line)
    print("-" * 64)
    free = (BENCH_LINKS - CLAMP_LINKS) * PITCH
    print(f" Bench test ({BENCH_LINKS} links, {BENCH_LINKS} pins):")
    print(f"   hang weights {free:.0f} mm out from the clamp and measure droop:")
    for g in (0, 250, 500, 1000):
        r = fork_load(g, 0, free, 0, head_g=0, pusher_t=0)
        print(f"   {g:5d} g  ->  {r['droop'] + r['play']:5.1f} mm predicted")
    print("   More droop than predicted: lower ETA (elastic) or raise PIN_CLEAR")
    print(f" Files in {out}/: link, sprocket_half, bench_clamp (.stl), bench_test.step")
    print("=" * 64)
