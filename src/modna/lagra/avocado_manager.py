"""
Avocado Manager - parametric concept model (build123d)
======================================================

Coordinate system (mm):
    X = width  (left -> right)
    Y = depth  (0 = front/customer, D = back/elevator)
    Z = height (0 = floor)

Run:    python avocado_manager.py
Output: avocado_manager.step  (opens in Fusion, Onshape, FreeCAD ...)

All dimensions are controlled by the parameters below. The script also
prints a capacity report, so you can see what actually fits.
"""

import math

from build123d import (
    Align,
    Box,
    Color,
    Compound,
    Cylinder,
    Pos,
    Rot,
    Sphere,
    export_step,
    scale,
)

# ---------------------------------------------------------------
# PARAMETERS
# ---------------------------------------------------------------
# Outer dimensions
W, D, H = 600, 600, 800
WALL = 25  # wall incl. insulation

# Avocado (Hass). The nominal size is what is drawn. Rows, level height, lip,
# fork and cradle are sized for the whole range, and the report checks
# capacity and the row end / fork function across it.
AVO_D = 65  # nominal diameter
AVO_L = 95  # nominal length
AVO_D_RANGE = (62, 78)  # min/max diameter (rough Hass range - measure your supply)
AVO_L_RANGE = (85, 115)  # min/max length
AVO_ROLLS = False  # True: long axis across the row (rolls)
# False: long axis along the row (slides)
CLEAR = 4  # clearance around the largest avocado

# Vertical layout
BASE_H = 120  # technical compartment (cooling/heating, electronics)
TOP_H = 130  # customer section on top

# Shelves
N_SHELVES = 6  # desired number (the script checks whether it fits)
SHELF_T = 20  # shelf thickness (insulated divider between zones)
SLOPE_DEG = 0.0  # slope down towards the front stops. 0 = flat: the fork puts
# every fruit where it stays. Use ~1-2 deg for fruit that rolls (AVO_ROLLS).
FRONT_CLEAR = 10  # gap between shelf and front wall
FILL = 0.7  # fraction of slots shown filled
LOAD_ROW = 3  # row on the bottom shelf that staff fill from the front (0 = none)

# Rows: what the fruit rests on. Stored fruit sit from the front stop
# backwards, newest nearest the elevator (LIFO), and never touch each other.
ROW_PROFILE = "v"  # "flat":  flat shelf plate between the row dividers
# "v":     lift-out V-channel tray per row (self-centring)
# "rails": two round rails per row (small contact, debris falls through)
V_ANGLE = 12  # "v": flank angle above horizontal (deg)
TRAY_T = 3  # "v": tray sheet thickness
RAIL_D = 10  # "rails": rail diameter
RAIL_GAP = 46  # "rails": centre distance between the two rails (fork fits between)
DIVIDER_T = 4  # row divider thickness
DIVIDER_H = 35  # row divider height above the shelf plate
STOP_T = 4  # front stop at the front end of each row
STOP_H = 45  # front stop height above the plate (above the fruit's centre)
PLACE_GAP = 5  # gap the fork leaves between stored fruit

# Loading row: staff push fruit in from the front against a fixed rear lip
# (two posts with a notch for the fork); the fork lifts them over it.
LIP_H = 8  # lip top above the resting (nominal) fruit's bottom
LIP_BACK = 3  # lip material behind the rear point of the fruit resting against it

# Fork on the carriage: two tines + pusher face, driven by a zip chain (two
# chains that zip into a rigid bar, and coil into a magazine when apart), so
# it reaches the front of a row from a shallow carriage. It is the only
# actuator on the carriage; the elevator's Z axis does the lifting. Each row
# has a groove along its whole length for the tines and the chain.
TINE_W = 5  # tine width
TINE_H = 8  # tine height
TINE_GAP = 22  # centre distance between the two tines
TINE_CLEAR = 3  # clearance tine <-> groove, and tine <-> fruit when sliding under
LIFT_CLEAR = 3  # a carried fruit clears the row floor (and loading lip) by this
TIP_MARGIN = 8  # the tip stays this far ahead of a fruit's centre and behind the next
PUSHER_T = 6  # pusher face thickness
PUSHER_H = 30  # pusher face height above the tines
CHAIN_W = 12  # zip chain width (runs between the tines)
CHAIN_H = 20  # zip chain height
ZIP_D = 30  # zipper + drive sprocket, at the back of the cradle
MAG_H = 70  # chain magazine under the zipper

# Elevator (rear side). The shaft depth follows from the carriage stack below.
ELEV_MODE = "scan"  # "fetch":  the fork has lifted the rearmost avocado of a row
# "insert": the fork carries an avocado to its place in the row
# "scan":   the carriage is up in the scan station
# "free":   the carriage is free-standing at ELEV_X / ELEV_Z
DOCK_SHELF = 2  # shelf the carriage is docked against (1 = lowest storage shelf)
DOCK_ROW = 2  # row the carriage is docked against (1 = left)
ELEV_X = 0.35  # only for "free" (0..1)
ELEV_Z = 0.55  # only for "free" (0..1)
FORK_SNAPSHOT = 0.5  # only for "insert": how far towards its place the avocado is

# Carriage stack, front to back: cradle pocket, pusher face, zipper, carriage
# plate, bearing blocks, Z rail, Z column
CR_GAP = 5  # gap between the shelves' rear edge and the cradle
CR_WALL_H = 50  # cradle side wall height above the tines
CARRIAGE_PLATE_T = 16
CARRIAGE_UP = 100  # carriage plate top above the dock origin
CARRIAGE_DOWN = 55  # carriage plate bottom below the dock origin
BEARING_T = 12
Z_RAIL_T = 6
COLUMN_T = 24

# Scan station at the top of the elevator shaft (multispectral camera + LEDs).
# The back of the machine is raised to make room for it above the height where
# the carriage delivers to the customer section.
SCAN_X = 0.5  # X position of the scan station (0..1 of the elevator's X travel)
CAM_SIDE = 0  # camera's sideways offset from the avocado centre (0 = top-down)
CAM_UP = 145  # camera's height above the avocado centre
CAM_BODY = (60, 60, 25)  # camera + LED holder: width, height, depth (view axis)
SCAN_CLEAR = 15  # clearance between camera and the carriage passing below it
ROOF_T = 20  # roof of the raised back

# Customer section on top
TOP_SECTIONS = ["Soft ripe", "Firm ripe", "Ripe in a few days"]
TOP_SLOPE_DEG = 6.0  # slopes towards the customer

# Temperature zones from bottom to top (colour/label only in the model)
ZONE_TEMPS = [7, 10, 13, 16, 19, 22, 24, 26]

SHOW_HOUSING = True

# ---------------------------------------------------------------
# DERIVED QUANTITIES
# ---------------------------------------------------------------
inner_w = W - 2 * WALL


def across_along(d, l):
    """Fruit size (across the row, along the row) for the chosen orientation."""
    return (l, d) if AVO_ROLLS else (d, l)


across_nom, along_nom = across_along(AVO_D, AVO_L)
across_min, along_min = across_along(AVO_D_RANGE[0], AVO_L_RANGE[0])
across_max, along_max = across_along(AVO_D_RANGE[1], AVO_L_RANGE[1])


def seat_height(across, height):
    """Fruit centre above the shelf plate. The fruit's cross-section across the
    row is treated as an ellipse (across x height) resting on the row profile."""
    a, b = across / 2, height / 2
    if ROW_PROFILE == "v":
        # V surfaces meet at the row centre, TRAY_T above the plate
        al = math.radians(V_ANGLE)
        return TRAY_T + math.hypot(a * math.sin(al), b * math.cos(al)) / math.cos(al)
    if ROW_PROFILE == "rails":
        # lowest centre height where the ellipse clears both rails (bisection)
        r, g = RAIL_D / 2, RAIL_GAP / 2
        ts = [2 * math.pi * i / 720 for i in range(720)]

        def clears(zc):
            return all(
                math.hypot(a * math.cos(t) - g, zc + b * math.sin(t) - r) >= r
                for t in ts
            )

        lo, hi = b, b + 2 * r
        if clears(lo):
            return lo
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if clears(mid) else (mid, hi)
        return hi
    return b


# Rows: sized for the largest fruit
row_inner = across_max + 2 * CLEAR
row_pitch = row_inner + DIVIDER_T
n_rows = int(inner_w // row_pitch)

# Fruit sizes checked below, as (across, along, height): smallest, nominal, largest
SIZES = [
    (*across_along(d, l), d)
    for d, l in (
        (AVO_D_RANGE[0], AVO_L_RANGE[0]),
        (AVO_D, AVO_L),
        (AVO_D_RANGE[1], AVO_L_RANGE[1]),
    )
]

# Fork groove along each row (and the notch between the loading row's lip posts)
notch = TINE_GAP / 2 + TINE_W / 2 + TINE_CLEAR  # half width of groove and notch
tine_in = TINE_GAP / 2 - TINE_W / 2  # inner edge of a tine (the fruit rests on it)


def on_points(across, height, u):
    """Centre of the fruit's cross-section above two support points at ±u."""
    a, b = across / 2, height / 2
    return b * math.sqrt(1 - (u / a) ** 2)


def row_seat(across, height):
    """Fruit centre above the plate, on the row profile with the fork groove
    cut out of it (the fruit may sink into the groove)."""
    if ROW_PROFILE == "v":
        al = math.radians(V_ANGLE)
        a, b = across / 2, height / 2
        u_t = a * a * math.sin(al) / math.hypot(a * math.sin(al), b * math.cos(al))
        if u_t > notch:  # still rests on the flanks, outside the groove
            return seat_height(across, height)
        return TRAY_T + notch * math.tan(al) + on_points(across, height, notch)
    if ROW_PROFILE == "rails":  # the groove lies between the rails
        return seat_height(across, height)
    return on_points(across, height, notch)


seat_nom = row_seat(across_nom, AVO_D)  # nominal fruit centre above plate
floor_z = seat_nom - AVO_D / 2  # nominal fruit bottom above plate
fruit_top_max = max(row_seat(ac, h) + h / 2 for ac, _, h in SIZES)

# Loading row lip
lip_top = floor_z + LIP_H  # above the plate


def lip_contact(across, along, height):
    """Fruit resting against the loading row's lip, touching the posts' top
    inner edges. Returns (bulge, hold): its rear bulges `bulge` mm past the
    posts' front face into the notch, and climbing the lip takes a push of
    `hold` x its weight (pivoting about the edge)."""
    a, l, b = across / 2, along / 2, height / 2
    dz = max(row_seat(across, height) - lip_top, 0.0)  # contact below the centre
    ext = l * math.sqrt(max(0.0, 1 - (notch / a) ** 2 - (dz / b) ** 2))
    return l - ext, ext / dz if dz > 0 else math.inf


contacts = [lip_contact(*s) for s in SIZES]
bulge_nom = contacts[1][0]
lip_d = max(bg for bg, _ in contacts) + LIP_BACK  # depth of the lip posts
hold_min = min(h for _, h in contacts)

# Fork heights above the plate. Low: the tines slide in under a fruit.
# High: a fruit on the tines (it sags between them) clears the row floor, and
# in the loading row the lip posts outside the notch.
tine_top_low = (
    min(row_seat(ac, h) - on_points(ac, h, tine_in) for ac, _, h in SIZES) - TINE_CLEAR
)
tine_top_high = max(
    row_seat(ac, h) + LIFT_CLEAR - on_points(ac, h, tine_in) for ac, _, h in SIZES
)
tine_top_high_load = max(
    lip_top + LIFT_CLEAR + on_points(ac, h, notch) - on_points(ac, h, tine_in)
    for ac, _, h in SIZES
)
lift = tine_top_high - tine_top_low  # Z move of the elevator per fetch/insert
lift_load = tine_top_high_load - tine_top_low  # ... in the loading row


def lifted_top(tine_top):
    """Top of the largest fruit on the tines, above the plate."""
    return max(tine_top + on_points(ac, h, tine_in) + h / 2 for ac, _, h in SIZES)


groove_d = TINE_H + TINE_CLEAR - tine_top_low  # groove depth below the plate

# Tine length. When fetching, the pusher face touches the fruit's rear (its
# size and place are known from the scan), so the tip must reach past the
# fruit's centre but stay behind the next fruit, for every fruit size.
tip_lo = along_max / 2 + TIP_MARGIN
tip_hi = along_min - TIP_MARGIN
tine_len = (tip_lo + tip_hi) / 2 if tip_lo <= tip_hi else tip_hi

# Carriage stack in v (backwards from the shelf's rear edge, v < 0 is inside
# the row): cradle pocket for the largest fruit, pusher face, zipper,
# carriage plate, Z guide
cr_v0 = CR_GAP  # cradle front
push_rest = cr_v0 + along_max + CLEAR  # pusher face at rest (back of the pocket)
cr_v1 = push_rest + PUSHER_T + ZIP_D  # back of cradle = carriage plate
v_bear = cr_v1 + CARRIAGE_PLATE_T
v_zrail = v_bear + BEARING_T
v_col = v_zrail + Z_RAIL_T
elev_depth = v_col + COLUMN_T  # depth of the elevator shaft

# Shelves
shelf_y0 = WALL + FRONT_CLEAR
shelf_y1 = D - WALL - elev_depth
shelf_len = shelf_y1 - shelf_y0
slope = math.radians(SLOPE_DEG)
drop = shelf_len * math.tan(slope)  # rear end above the front
stop_face = shelf_len - lip_d  # front face of the loading row's lip posts
q_rear_load = stop_face + bulge_nom  # rear point of the fruit against the lip
v_deep = STOP_T + along_min + 2 - shelf_len  # pusher face at the frontmost fruit
fork_reach = push_rest - v_deep  # fork stroke, rest -> front of the row
groove_y0 = max(STOP_T + 2, shelf_len + v_deep - tine_len - 5)  # groove front end


def per_row(along):
    return int((shelf_len - STOP_T + PLACE_GAP) // (along + PLACE_GAP))


def per_load_row(along):
    return int(q_rear_load // along)


def queue(is_load, n):
    """Centres (ly) of n nominal fruit in a row, front to rear. Stored fruit
    sit from the front stop with gaps; the loading row is pushed in by hand
    against the rear lip."""
    if is_load:
        return [q_rear_load - along_nom / 2 - k * along_nom for k in range(n)][::-1]
    return [STOP_T + along_nom / 2 + k * (along_nom + PLACE_GAP) for k in range(n)]


per_row_nom = per_row(along_nom)

load_row = LOAD_ROW if 1 <= LOAD_ROW <= n_rows else 0
level_pitch = max(fruit_top_max, lifted_top(tine_top_high)) + CLEAR + SHELF_T
# the bottom level is taller if the loading row's lift needs it
level1_pitch = max(
    level_pitch,
    lifted_top(tine_top_high_load) + CLEAR + SHELF_T if load_row else 0,
)
storage_z0 = BASE_H
storage_z1 = H - TOP_H
avail_h = storage_z1 - storage_z0 - drop
n_levels_fit = (
    1 + int((avail_h - level1_pitch) // level_pitch) if avail_h >= level1_pitch else 0
)
n_shelves = min(N_SHELVES, n_levels_fit)

# Customer section on top
top_z = H - TOP_H + 20
top_len = shelf_y1 - WALL - 5
top_slope = math.radians(TOP_SLOPE_DEG)
top_drop = top_len * math.tan(top_slope)


# Elevator travel. The carriage is built around a dock origin at the shelf's
# rear edge (row centre, plate surface): there the fork is low, `lift` higher
# it is high. The fork and cradle are tilted like the shelves (level when
# SLOPE_DEG = 0); fork_to_carriage() maps their (v, w) to the upright
# carriage frame.
def fork_to_carriage(v, w):
    return (
        v * math.cos(slope) - w * math.sin(slope),
        v * math.sin(slope) + w * math.cos(slope),
    )


def fruit_on_tines(across, along, height, face):
    """(v, w) of a fruit's centre on the tines, rear against the pusher face."""
    return face - along / 2, tine_top_low + on_points(across, height, tine_in)


cr_w = row_inner  # cradle inner width
cradle_avo = (0, *fork_to_carriage(*fruit_on_tines(*SIZES[1], push_rest)))
cradle_top = max(  # top of the largest fruit
    fork_to_carriage(*fruit_on_tines(ac, al, h, push_rest))[1] + h / 2
    for ac, al, h in SIZES
)
x_travel0 = WALL + 30
x_travel1 = W - WALL - 30
z_travel0 = storage_z0
z_deliver = top_z + top_len * math.sin(top_slope)  # rear end of the top trays

# Scan station: the camera looks down (or sideways/down if CAM_SIDE > 0) into
# the cradle. Its lowest point must clear the avocado in the cradle when the
# carriage passes below it on its way to the top section (fork high).
cam_tilt = math.atan2(CAM_UP, CAM_SIDE)  # below horizontal
cam_half_w = CAM_BODY[0] / 2 * math.cos(cam_tilt) + CAM_BODY[2] / 2 * math.sin(cam_tilt)
lens_reach = (CAM_BODY[2] / 2 + 6) * math.sin(cam_tilt)  # lens tip
cam_lo = cradle_avo[2] + CAM_UP - max(cam_half_w, lens_reach)  # rel. dock origin
cam_hi = cradle_avo[2] + CAM_UP + cam_half_w
scan_x = x_travel0 + SCAN_X * (x_travel1 - x_travel0)
z_scan = z_deliver + lift + cradle_top + SCAN_CLEAR - cam_lo
z_travel1 = max(z_scan, z_deliver + lift)  # highest dock origin
z_rail = z_travel1 + CARRIAGE_UP + SCAN_CLEAR  # upper X rail (bottom)
h_back = max(z_rail + 18, z_scan + cam_hi + 10) + ROOF_T

# ---------------------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------------------
parts = []


def add(shape, label, color):
    shape.label = label
    shape.color = Color(color)
    parts.append(shape)
    return shape


def box(x, y, z, dx, dy, dz):
    """Box from corner (x,y,z) with size (dx,dy,dz)."""
    return Pos(x, y, z) * Box(dx, dy, dz, align=(Align.MIN, Align.MIN, Align.MIN))


def avocado_shape(along_y=not AVO_ROLLS):
    """Nominal avocado centred at the origin, long axis along X (or Y)."""
    a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
    return Rot(0, 0, 90) * a if along_y else a


def zone_color(i, n):
    # blue (cold) -> orange (warm)
    t = i / max(n - 1, 1)
    r = 0.25 + 0.7 * t
    g = 0.45 + 0.1 * (1 - abs(0.5 - t) * 2)
    b = 0.85 - 0.65 * t
    return (r, g, b)


RIPE_COLORS = ["#2f3a1c", "#3f5a22", "#557a2b", "#6b9a33", "#86b83e", "#9fcf4c"]

# ---------------------------------------------------------------
# HOUSING
# ---------------------------------------------------------------
if SHOW_HOUSING:
    add(box(0, 0, 0, WALL, D, H), "Side wall L", "#d9d9d9")
    add(box(W - WALL, 0, 0, WALL, D, H), "Side wall R", "#d9d9d9")
    add(box(WALL, 0, 0, inner_w, WALL, BASE_H), "Front plinth", "#bdbdbd")
    # raised back over the elevator shaft, houses the scan station
    back_y0 = shelf_y1 - WALL
    back_d = D - back_y0
    add(box(WALL, D - WALL, 0, inner_w, WALL, h_back - ROOF_T), "Rear wall", "#cfcfcf")
    add(box(0, back_y0, H, WALL, back_d, h_back - H), "Raised back – side L", "#d9d9d9")
    add(
        box(W - WALL, back_y0, H, WALL, back_d, h_back - H),
        "Raised back – side R",
        "#d9d9d9",
    )
    add(
        box(WALL, back_y0, H, inner_w, WALL, h_back - ROOF_T - H),
        "Raised back – front",
        "#cfcfcf",
    )
    add(
        box(0, back_y0, h_back - ROOF_T, W, back_d, ROOF_T),
        "Raised back – roof",
        "#cfcfcf",
    )
# stops at the elevator shaft, so the carriage can reach the bottom shelf
add(
    box(WALL, WALL, 0, inner_w, shelf_y1 - WALL, BASE_H - 5),
    "Technical compartment (cooling/heating, control)",
    "#7a7a7a",
)

z = storage_z0

# ---------------------------------------------------------------
# STORAGE SHELVES (no moving parts)
# ---------------------------------------------------------------
# Each shelf is built in a local coordinate system that follows the shelf plate:
#   lx = 0..inner_w (width), ly = 0 front .. shelf_len rear (elevator),
#   lz = 0 at the shelf plate's surface. Then everything is tilted SLOPE_DEG
#   (rear end up).
#
# Stored rows: fruit sit from a fixed front stop backwards, newest nearest the
# elevator. A groove along the whole row, open to the rear, takes the fork's
# tines and chain. The software keeps every row's contents (place and size
# of each fruit, from the scan), so the fork only ever handles the rearmost:
#   Fetch:  fork low -> tines slide in under the rearmost fruit -> the
#           elevator lifts `lift` mm -> fork retracts. Nothing else moves.
#   Insert: fork high -> carries the fruit to PLACE_GAP behind the rearmost
#           one -> the elevator lowers, the tines drop into the groove ->
#           fork retracts under it.
# Loading row (bottom shelf): staff push fruit in from the front against a
# fixed rear lip; the fork fetches from it the same way, lifting `lift_load`
# to clear the lip. The next fruit's place follows from the fetched fruit's
# length, measured at the scan.

rows_w = n_rows * row_pitch
x_off = WALL + (inner_w - rows_w) / 2
a_sl = slope  # angle for local -> global


def to_world(base, lx, ly, lz):
    """Local shelf point -> global coordinates."""
    return (
        WALL + lx,
        shelf_y0 + ly * math.cos(a_sl) - lz * math.sin(a_sl),
        base + ly * math.sin(a_sl) + lz * math.cos(a_sl),
    )


def row_cx(r):
    return x_off - WALL + r * row_pitch + row_pitch / 2


def groove(cxl, z0=-groove_d, dz=60):
    """Groove for the fork along a row (plate and tray)."""
    return box(cxl - notch, groove_y0, z0, 2 * notch, shelf_len - groove_y0 + 1, dz)


def row_floor(cxl, length):
    """What the fruit rests on in one row, besides the shelf plate."""
    if ROW_PROFILE == "v":
        w = row_inner / 2 / math.cos(math.radians(V_ANGLE))
        out = []
        for sgn in (-1, 1):
            flank = box(
                0 if sgn > 0 else -w, STOP_T, -TRAY_T, w, length - STOP_T, TRAY_T
            )
            flank = Pos(cxl, 0, TRAY_T) * Rot(0, -sgn * V_ANGLE, 0) * flank
            flank -= groove(cxl)
            out.append((flank, "Row tray (V-channel, lift-out)", "#e0f2f1"))
        return out
    if ROW_PROFILE == "rails":
        return [
            (
                Pos(cxl + sgn * RAIL_GAP / 2, length / 2, RAIL_D / 2)
                * Rot(90, 0, 0)
                * Cylinder(RAIL_D / 2, length),
                "Row rail",
                "#e0f2f1",
            )
            for sgn in (-1, 1)
        ]
    return []


def front_stop(cxl):
    return [
        (
            box(cxl - row_inner / 2, 0, 0, row_inner, STOP_T, STOP_H),
            "Row front stop",
            "#c0392b",
        )
    ]


def load_lip(cxl):
    """The loading row's fixed lip: two posts either side of the fork notch."""
    post_w = row_inner / 2 - notch
    return [
        (
            box(
                cxl + notch if sgn > 0 else cxl - row_inner / 2,
                stop_face,
                0,
                post_w,
                lip_d,
                lip_top,
            ),
            "Loading row lip (fixed)",
            "#c0392b",
        )
        for sgn in (-1, 1)
    ]


docked = ELEV_MODE in ("fetch", "insert")
dock_origin = None
dock_is_load = False
push_face = push_rest  # pusher face in the snapshot (v)

for s in range(n_shelves):
    zc = zone_color(s, n_shelves)
    temp = ZONE_TEMPS[s] if s < len(ZONE_TEMPS) else "?"
    base = z  # front edge (lowest point)
    T = Pos(WALL, shelf_y0, base) * Rot(SLOPE_DEG, 0, 0)
    local = []

    # Shelf plate with a fork groove per row
    plate = box(0, 0, -SHELF_T, inner_w, shelf_len, SHELF_T)
    for r in range(n_rows):
        plate -= groove(row_cx(r))
    local.append((plate, f"Shelf {s + 1} ({temp} °C)", zc))

    # Row dividers
    for r in range(n_rows + 1):
        rx = x_off - WALL + r * row_pitch - DIVIDER_T / 2
        local.append(
            (box(rx, 0, 0, DIVIDER_T, shelf_len, DIVIDER_H), "Row divider", "#eeeeee")
        )

    # Row floors, front stops / loading lip + avocados
    n_fill = min(round(per_row_nom * FILL), per_row_nom - 1)  # leave room to insert
    for r in range(n_rows):
        cxl = row_cx(r)
        is_load = s == 0 and r == load_row - 1
        is_dock = docked and s == DOCK_SHELF - 1 and r == DOCK_ROW - 1
        if is_load:
            local += row_floor(cxl, stop_face) + load_lip(cxl)
            n = min(round(per_load_row(along_nom) * FILL), per_load_row(along_nom))
            col = "#7d8a5c"  # not yet scanned
        else:
            local += row_floor(cxl, shelf_len) + front_stop(cxl)
            n = n_fill
            col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        cys = queue(is_load, n)
        if is_dock:
            dock_origin = to_world(base, cxl, shelf_len, 0)
            dock_is_load = is_load
            if ELEV_MODE == "fetch" and cys:  # the rearmost is on the fork
                rear = q_rear_load if is_load else cys[-1] + along_nom / 2
                push_face = rear + 2 - shelf_len
                cys = cys[:-1]
            elif ELEV_MODE == "insert" and not is_load:  # on its way to its place
                place = STOP_T + n * (along_nom + PLACE_GAP) + along_nom - shelf_len
                push_face = push_rest + FORK_SNAPSHOT * (place - push_rest)
        lab = "Avocado (loaded, not scanned)" if is_load else "Avocado"
        for cy in cys:
            local.append((Pos(cxl, cy, seat_nom) * avocado_shape(), lab, col))
        if is_load:
            # opening in the front where staff fill the loading row
            for fz in (-SHELF_T - 10, fruit_top_max + 12):
                local.append(
                    (
                        box(
                            cxl - row_pitch / 2,
                            -FRONT_CLEAR - WALL,
                            fz,
                            row_pitch,
                            WALL,
                            10,
                        ),
                        "Loading opening – frame",
                        "#444444",
                    )
                )

    for shp, lab, col in local:
        add(T * shp, lab, col)

    z += level1_pitch if s == 0 else level_pitch

# ---------------------------------------------------------------
# CUSTOMER SECTION ON TOP
# ---------------------------------------------------------------
sec_w = inner_w / len(TOP_SECTIONS)
top_colors = ["#2f3a1c", "#557a2b", "#86b83e"]

for i, name in enumerate(TOP_SECTIONS):
    sx = WALL + i * sec_w
    # slot at the rear end, where the fork lowers the fruit onto the tray
    tray = box(0, 0, -12, sec_w - 6, top_len, 12) - box(
        (sec_w - 6) / 2 - notch, top_len - tine_len - 20, -13, 2 * notch, top_len, 14
    )
    tray = Pos(sx + 3, WALL, top_z) * Rot(TOP_SLOPE_DEG, 0, 0) * tray
    add(tray, f"Top section: {name}", "#8d6e63")
    # low front edge + divider
    add(
        box(sx + 3, WALL - 10, top_z - 12, sec_w - 6, 10, 45),
        f"Front edge {name}",
        "#5d4037",
    )
    div = Pos(sx, WALL, top_z) * Rot(TOP_SLOPE_DEG, 0, 0) * box(0, 0, 0, 6, top_len, 40)
    add(div, "Top divider", "#5d4037")
    # avocados gathered at the front (rolling towards the customer)
    n_side = int((sec_w - 20) // (AVO_D + 5))
    for rr in range(2):
        for c in range(n_side):
            ax = sx + 10 + (AVO_D + 5) / 2 + c * (AVO_D + 5)
            ay = WALL + AVO_L / 2 + 5 + rr * (AVO_L + 5)
            az = top_z + (ay - WALL) * math.tan(top_slope) + AVO_D / 2
            add(Pos(ax, ay, az) * avocado_shape(along_y=True), "Avocado", top_colors[i])


# ---------------------------------------------------------------
# ELEVATOR (X-Z gantry at the rear) WITH FORK CARRIAGE
# ---------------------------------------------------------------
# The carriage is built in its own system (u, v, w) with origin at the shelf's
# rear edge (row centre, surface) when the carriage is docked with the fork low:
#   u = sideways (X), v = backwards into the shaft (Y), w = up (Z)
# The fork and cradle use the same system tilted SLOPE_DEG like the shelves
# (frame F), so the fork runs parallel to the row floor.
#
# The fork is the only actuator on the carriage: two tines with a pusher face,
# pushed and pulled by a zip chain. The chain's two halves separate in the
# zipper at the back of the cradle and coil into the magazine below it, so a
# shallow carriage reaches the front of a row. Lifting is done by the Z axis.
# Fetch / insert: see the shelf section. The snapshots show the fork high,
#   with the rearmost avocado lifted, or on its way to its place.
# Deliver: the fork, high, reaches over the rear end of a top tray, the
#   elevator lowers so the tines pass through the tray's slot, and the fruit
#   rolls down to the customer.
# Scan (ELEV_MODE = "scan"): the carriage waits under the camera at SCAN_X.
#   Nothing is above the cradle, so the camera sees the whole fruit. Besides
#   ripeness, the scan gives each fruit's size, which sets where it goes in a
#   row and where the fork picks it up again. Any fetched fruit can be driven
#   past the camera again to check it is the expected one.

lifted = docked and dock_origin is not None  # fork high at a row
if ELEV_MODE == "scan":
    dock_origin = (scan_x, shelf_y1, z_scan)
if dock_origin is None:  # "free" or invalid dock position
    dock_origin = (
        x_travel0 + ELEV_X * (x_travel1 - x_travel0),
        shelf_y1,
        z_travel0 + ELEV_Z * (z_travel1 - z_travel0),
    )
ox, oy, oz = dock_origin
C = Pos(ox, oy, oz + ((lift_load if dock_is_load else lift) if lifted else 0))
F = C * Rot(SLOPE_DEG, 0, 0)


def cbox(u, v, w, du, dv, dw):
    return C * box(u, v, w, du, dv, dw)


def fbox(u, v, w, du, dv, dw):
    return F * box(u, v, w, du, dv, dw)


# Gantry: X rails at the bottom and top, Z column with linear rail
for zr in (z_travel0 - 30, z_rail):
    add(box(WALL, D - WALL - COLUMN_T, zr, inner_w, COLUMN_T, 18), "X rail", "#607d8b")
col_z0, col_z1 = z_travel0 - 12, z_rail
add(
    box(ox - 20, oy + v_col, col_z0, 40, COLUMN_T, col_z1 - col_z0),
    "Z actuator (column)",
    "#455a64",
)
add(
    box(ox - 8, oy + v_zrail, col_z0, 16, Z_RAIL_T, col_z1 - col_z0),
    "Z linear rail",
    "#b0bec5",
)

# Carriage
plate_w = cr_w + 8
add(
    cbox(
        -plate_w / 2,
        cr_v1,
        -CARRIAGE_DOWN,
        plate_w,
        CARRIAGE_PLATE_T,
        CARRIAGE_UP + CARRIAGE_DOWN,
    ),
    "Carriage plate",
    "#37474f",
)
for w0 in (-CARRIAGE_DOWN + 10, CARRIAGE_UP - 45):
    add(cbox(-22, v_bear, w0, 44, BEARING_T, 30), "Bearing block", "#263238")

# Cradle: base the fork slides on, and side walls. There is no cradle floor -
# the fruit rides on the tines.
tt = tine_top_low  # tine top
w_fork = tt - TINE_H  # underside of tines and chain
zip_v = cr_v1 - ZIP_D  # front of the zipper
add(
    fbox(-cr_w / 2, cr_v0, w_fork - 6, cr_w, zip_v - cr_v0, 5),
    "Cradle – base",
    "#ffb300",
)
for sgn in (-1, 1):
    u0 = cr_w / 2 if sgn > 0 else -cr_w / 2 - 4
    add(
        fbox(u0, cr_v0, w_fork - 6, 4, cr_v1 - cr_v0 - 3, tt + CR_WALL_H - w_fork + 6),
        "Cradle – side wall",
        "#ffa000",
    )
add(
    fbox(-cr_w / 2 - 12, push_rest - 20, tt + 12, 8, 14, 10),
    "Sensor: avocado in cradle",
    "#212121",
)

# Fork: tines + pusher face, pushed by the zip chain. Everything that enters a
# row under a fruit stays inside the groove.
fork_w = TINE_GAP + TINE_W
for sgn in (-1, 1):
    add(
        fbox(
            sgn * TINE_GAP / 2 - TINE_W / 2,
            push_face - tine_len,
            w_fork,
            TINE_W,
            tine_len,
            TINE_H,
        ),
        "Fork – tine",
        "#9e9e9e",
    )
add(
    fbox(-fork_w / 2, push_face, w_fork, fork_w, PUSHER_T, TINE_H + PUSHER_H),
    "Fork – pusher face",
    "#f4511e",
)
chain_len = zip_v - (push_face + PUSHER_T)
if chain_len > 0.5:
    add(
        fbox(-CHAIN_W / 2, push_face + PUSHER_T, w_fork, CHAIN_W, chain_len, CHAIN_H),
        "Fork – zip chain (zipped, rigid)",
        "#78909c",
    )
add(
    fbox(-20, zip_v, w_fork - 6, 40, ZIP_D - 3, CHAIN_H + 12),
    "Zip-chain drive (zipper + motor)",
    "#37474f",
)
add(
    fbox(-22, zip_v, w_fork - 6 - MAG_H, 44, ZIP_D - 3, MAG_H),
    "Chain magazine (halves apart)",
    "#546e7a",
)

# Avocado on the fork: resting in the cradle, lifted out of a row (rear just
# ahead of the pusher face), or on its way to its place in a row
face = push_face - 2 if ELEV_MODE == "fetch" and lifted else push_face
add(
    F * Pos(0, *fruit_on_tines(*SIZES[1], face)) * avocado_shape(),
    "Avocado",
    "#6b9a33",
)

# ---------------------------------------------------------------
# SCAN STATION (multispectral camera + LEDs in the raised back)
# ---------------------------------------------------------------
# Built in the view system: the camera looks along -z, then it is tilted
# cam_tilt below horizontal, looking towards -X into the cradle.
cw, ch, cd = CAM_BODY
cam = [
    (Box(cw, ch, cd), "Multispectral camera (Pi + LED holder)", "#1f1f1f"),
    (Pos(0, 0, -cd / 2 - 3) * Cylinder(8, 6), "Camera lens", "#3050a0"),
]
for lx, ly in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
    cam.append(
        (
            Pos(lx * (cw / 2 - 8), ly * (ch / 2 - 8), -cd / 2 - 1) * Cylinder(3, 2),
            "Scan LED",
            "#fff59d",
        )
    )
cam_c = (scan_x + CAM_SIDE, shelf_y1 + cradle_avo[1], z_scan + cradle_avo[2] + CAM_UP)
T = Pos(*cam_c) * Rot(0, 90 - math.degrees(cam_tilt), 0)
for shp, lab, col in cam:
    add(T * shp, lab, col)
cam_top = z_scan + cam_hi
add(
    box(
        cam_c[0] - 6,
        cam_c[1] - 10,
        cam_top - 10,
        12,
        20,
        h_back - ROOF_T - cam_top + 10,
    ),
    "Camera bracket",
    "#78909c",
)

# ---------------------------------------------------------------
# EXPORT + REPORT
# ---------------------------------------------------------------
model = Compound(children=parts, label="Avocado Manager")

if __name__ == "__main__":
    export_step(model, "avocado_manager.step")

    print("=" * 56)
    print(" FRUIT AND ROWS")
    print("=" * 56)
    print(
        f" Orientation / profile:     {'rolls' if AVO_ROLLS else 'slides'} / {ROW_PROFILE}"
        + (f", {SLOPE_DEG}° to the front" if SLOPE_DEG else ", flat")
    )
    print(
        f" Fruit size range:          D {AVO_D_RANGE[0]}–{AVO_D_RANGE[1]},"
        f" L {AVO_L_RANGE[0]}–{AVO_L_RANGE[1]} mm  (nominal {AVO_D} x {AVO_L})"
    )
    print(
        f" Row inner width:           {row_inner:.0f} mm  (largest fruit + clearance)"
    )
    print(
        f" Divider height:            {DIVIDER_H} mm"
        f"  ({DIVIDER_H / AVO_D_RANGE[1]:.2f} x largest diameter)"
    )
    print(f" Fruit bottom above plate:  {floor_z:.1f} mm  (nominal fruit)")
    if ROW_PROFILE == "rails" and RAIL_GAP - RAIL_D < 2 * notch:
        print("   ! the fork groove does not fit between the rails")
    print(
        f" Fork groove:               {2 * notch:.0f} wide x {groove_d:.0f} deep"
        f"  (in a {SHELF_T} mm shelf)"
    )
    if groove_d > SHELF_T - 4:
        print("   ! the groove leaves less than 4 mm of shelf under it")
    print("=" * 56)
    print(" FORK AND ELEVATOR")
    print("=" * 56)
    print(
        f" Tine length:               {tine_len:.0f} mm  (window"
        f" {tip_lo:.0f}–{tip_hi:.0f} over the size range)"
    )
    if tip_lo > tip_hi:
        print("   ! size range too wide: long fruit overhang the tip past their centre")
    print(
        f" Lift per fetch / insert:   {lift:.0f} mm, loading row {lift_load:.0f} mm"
        "  (elevator Z)"
    )
    print(f" Fork reach (zip chain):    {fork_reach:.0f} mm  (rest -> front of a row)")
    print(f" Cradle pocket (W x D):     {cr_w:.0f} x {push_rest - cr_v0:.0f} mm")
    print(f" Elevator shaft depth:      {elev_depth:.0f} mm")
    if load_row:
        lip_rel = [lip_top - (row_seat(ac, h) - h / 2) for ac, _, h in SIZES]
        print(
            f" Loading lip:               {min(lip_rel):.0f}–{max(lip_rel):.0f} mm"
            f" above fruit bottom, climbing takes {hold_min:.2f} x fruit weight"
        )
    print("=" * 56)
    print(" SCAN STATION")
    print("=" * 56)
    print(f" Height at the back:        {h_back:.0f} mm  (front {H} mm)")
    print(f" Top-section delivery:      z = {z_deliver:.0f} mm")
    print(f" Scan position:             z = {z_scan:.0f} mm")
    print(
        f" Camera tilt / lens gap:    {math.degrees(cam_tilt):.0f}° / "
        f"{math.hypot(CAM_SIDE, CAM_UP) - CAM_BODY[2] / 2 - 6 - AVO_D / 2:.0f} mm"
        " to avocado surface"
    )
    print("=" * 56)
    print(" CAPACITY REPORT")
    print("=" * 56)
    print(f" Shelf area (W x D):        {inner_w:.0f} x {shelf_len:.0f} mm")
    print(f" Rows per shelf:            {n_rows}  (row pitch {row_pitch:.0f} mm)")
    print(
        f" Avocados per row:          {per_row_nom}"
        f"  (smallest fruit {per_row(along_min)}, largest {per_row(along_max)})"
    )
    print(f" Avocados per shelf:        {n_rows * per_row_nom}")
    print(
        f" Level height:              {level_pitch:.0f} mm"
        + (f"  (bottom level {level1_pitch:.0f})" if level1_pitch > level_pitch else "")
    )
    print(
        f" Levels that fit:           {n_levels_fit}"
        + (f"  (row {load_row} on shelf 1 used for loading)" if load_row else "")
    )
    print(f" Storage shelves in model:  {n_shelves}  (desired {N_SHELVES})")
    load_cap = per_row_nom if load_row else 0
    print(f" Total storage capacity:    {n_shelves * n_rows * per_row_nom - load_cap}")
    print(
        f"   (all largest fruit:      "
        f"{n_shelves * n_rows * per_row(along_max) - (per_row(along_max) if load_row else 0)})"
    )
    if load_row:
        print(f" Loading row buffer:        {per_load_row(along_nom)}")
    if n_shelves < N_SHELVES:
        need = level1_pitch + (N_SHELVES - 1) * level_pitch + drop + BASE_H + TOP_H
        print(f" ! {N_SHELVES} shelves require approx. {need:.0f} mm total height.")
    print("=" * 56)

    # -----------------------------------------------------------
    # SHOW IN VS CODE (OCP CAD Viewer)
    # -----------------------------------------------------------
    # Requires the "OCP CAD Viewer" extension in VS Code and:
    #   pip install ocp_vscode
    # The viewer panel must be open (click the OCP icon in VS Code).
    # Parts are shown with names and colours, and can be hidden in the tree view.
    try:
        from ocp_vscode import Camera, set_defaults, show

        set_defaults(reset_camera=Camera.KEEP)  # keep camera angle on re-run
        show(model)
    except ImportError:
        print(" (ocp_vscode not installed – skipping 3D view)")
    except Exception as e:  # noqa: BLE001 - e.g. the viewer panel is not open
        print(f" (Could not show in OCP CAD Viewer: {e})")
