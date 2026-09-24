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

# Avocado (Hass). The nominal size is what is drawn and what the gate is tuned
# for. Rows, level height and cradle are sized for the largest fruit, and the
# report checks capacity and gate function across the whole range.
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
SLOPE_DEG = 3.0  # downward slope towards the elevator
FRONT_CLEAR = 10  # gap between shelf and front wall
GATE_SPACE = 25  # space under the shelf for the gate mechanism
FILL = 0.7  # fraction of slots shown filled
LOAD_ROW = 3  # row on the bottom shelf that staff fill from the front (0 = none)

# Rows: what the fruit rests on
ROW_PROFILE = "v"  # "flat":  flat shelf plate between the row dividers
# "v":     lift-out V-channel tray per row (self-centring)
# "rails": two round rails per row (small contact, debris falls through)
V_ANGLE = 12  # "v": flank angle above horizontal (deg)
TRAY_T = 3  # "v": tray sheet thickness
RAIL_D = 10  # "rails": rail diameter
RAIL_GAP = 40  # "rails": centre distance between the two rails
DIVIDER_T = 4  # row divider thickness
DIVIDER_H = 35  # row divider height above the shelf plate
HOLD_DOWN = False  # rail above each row, stops the queue climbing during insert
HOLD_T = 6  # hold-down rail thickness

# Gate (escapement) at the rear end of each row
GATE_TRAVEL = 19  # vertical travel of the stops (mm)
GATE_FLAP_H = 17  # height of the rear stop above the resting fruit's bottom
GATE_FLAP_W = 24  # width of the rear stop flap (must fit between rails)
GATE_PIN_D = 8  # diameter of the front pin
GATE_TAB = 12  # how far the rocker arm's roller protrudes into the elevator shaft

# Elevator (rear side). The shaft depth follows from the carriage stack below.
ELEV_MODE = "scan"  # "fetch":  the carriage is docked and releases one avocado
# "insert": the pusher pushes an avocado into the row
# "scan":   the carriage is up in the scan station
# "free":   the carriage is free-standing at ELEV_X / ELEV_Z
DOCK_SHELF = 2  # shelf the carriage is docked against (1 = lowest storage shelf)
DOCK_ROW = 2  # row the carriage is docked against (1 = left)
ELEV_X = 0.35  # only for "free" (0..1)
ELEV_Z = 0.55  # only for "free" (0..1)
PUSH_SNAPSHOT = 0.5  # only for "insert": how far through its stroke the pusher is

# Carriage stack, front to back: cradle pocket, pusher, carriage plate,
# bearing blocks, Z rail, Z column
CR_DROP = 8  # cradle floor below the fruit's resting bottom on the shelf
CR_FLOOR_T = 5  # cradle floor thickness
CR_WALL_H = 50  # cradle side wall height
PUSH_BACK = 16  # pusher plate + arm behind the fruit pocket
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


seat_nom = seat_height(across_nom, AVO_D)  # nominal fruit centre above plate
floor_z = seat_nom - AVO_D / 2  # nominal fruit bottom above plate
fruit_top_max = seat_height(across_max, AVO_D_RANGE[1]) + AVO_D_RANGE[1] / 2
hold_z = fruit_top_max + CLEAR  # underside of the hold-down rail

# Rows: sized for the largest fruit
row_inner = across_max + 2 * CLEAR
row_pitch = row_inner + DIVIDER_T
n_rows = int(inner_w // row_pitch)

# Carriage stack in v (backwards from the shelf's rear edge): gate rollers,
# cradle pocket for the largest fruit, pusher, carriage plate, Z guide
cr_v0 = GATE_TAB + 4  # cradle front, clear of the gate rollers
push_rest = cr_v0 + along_max + CLEAR  # pusher face at rest (back of the pocket)
cr_v1 = push_rest + PUSH_BACK  # back of the cradle = front of the carriage plate
v_bear = cr_v1 + CARRIAGE_PLATE_T
v_zrail = v_bear + BEARING_T
v_col = v_zrail + Z_RAIL_T
elev_depth = v_col + COLUMN_T  # depth of the elevator shaft

# Shelves
shelf_y0 = WALL + FRONT_CLEAR
shelf_y1 = D - WALL - elev_depth
shelf_len = shelf_y1 - shelf_y0
slope = math.radians(SLOPE_DEG)
drop = shelf_len * math.tan(slope)
stop_face = shelf_len - 6  # front face of the rear stop = end of the queue
slot_len = along_nom + 2  # drawn spacing in the queue


def per_row(along):
    return int(stop_face // along)


per_row_nom = per_row(along_nom)

level_pitch = (
    fruit_top_max + CLEAR + (HOLD_T if HOLD_DOWN else 0) + SHELF_T + GATE_SPACE
)
storage_z0 = BASE_H
storage_z1 = H - TOP_H
n_levels_fit = int((storage_z1 - storage_z0 - drop) // level_pitch)
n_shelves = min(N_SHELVES, n_levels_fit)
load_row = LOAD_ROW if 1 <= LOAD_ROW <= n_rows else 0

# Front pin vs fruit size: the pin rises into the wedge-shaped gap where fruit
# 1 and 2 touch. The gap is computed for two nominal fruits; the contact point
# moves when fruit 1 is smaller or larger than nominal.
pin_h = GATE_TRAVEL - 2  # raised pin tip above the fruit's bottom
_b = AVO_D / 2
pin_gap = along_nom * (1 - math.sqrt(max(0.0, 1 - ((_b - pin_h) / _b) ** 2)))
pin_tol = pin_gap / 2 - GATE_PIN_D / 2  # allowed contact point offset
pin_shift = max(along_nom - along_min, along_max - along_nom)

# Pusher: from rest to just past the rear flap (v < 0 is inside the row)
push_end = stop_face - shelf_len - 2
if ELEV_MODE == "insert":
    push_face = push_rest + PUSH_SNAPSHOT * (push_end - push_rest)
else:
    push_face = push_rest
# how far the new fruit has pushed the queue up the slope
queue_shift = max(0.0, stop_face - (shelf_len + push_face - along_nom))
PUSH_U = -(GATE_FLAP_W / 2 + 8)  # pusher arm offset, clears the rear post/flap
push_w = min(80, row_inner - 10)  # pusher plate width, fits inside a row

# Customer section on top
top_z = H - TOP_H + 20
top_len = shelf_y1 - WALL - 5
top_slope = math.radians(TOP_SLOPE_DEG)
top_drop = top_len * math.tan(top_slope)

# Elevator travel. The carriage is built around a dock origin at the
# shelf's rear edge (see ELEVATOR); an avocado in the cradle sits at
# cradle_avo (u, v, w) relative to it. Above the cradle itself nothing is
# higher than the avocado, so the camera can sit there.
cr_w = row_inner  # cradle inner width
cradle_floor = floor_z - CR_DROP  # w of the cradle floor surface
cradle_avo = (0, push_rest - along_nom / 2, cradle_floor + AVO_D / 2)
cradle_top = cradle_floor + AVO_D_RANGE[1]  # top of the largest fruit
x_travel0 = WALL + 30
x_travel1 = W - WALL - 30
z_travel0 = storage_z0
z_deliver = top_z + top_len * math.sin(top_slope)  # rear end of the top trays

# Scan station: the camera looks down (or sideways/down if CAM_SIDE > 0) into
# the cradle. Its lowest point must clear the avocado in the cradle when the
# carriage delivers to the top section below it.
cam_tilt = math.atan2(CAM_UP, CAM_SIDE)  # below horizontal
cam_half_w = CAM_BODY[0] / 2 * math.cos(cam_tilt) + CAM_BODY[2] / 2 * math.sin(cam_tilt)
lens_reach = (CAM_BODY[2] / 2 + 6) * math.sin(cam_tilt)  # lens tip
cam_lo = cradle_avo[2] + CAM_UP - max(cam_half_w, lens_reach)  # rel. dock origin
cam_hi = cradle_avo[2] + CAM_UP + cam_half_w
scan_x = x_travel0 + SCAN_X * (x_travel1 - x_travel0)
z_scan = z_deliver + cradle_top + SCAN_CLEAR - cam_lo
z_travel1 = max(z_scan, z_deliver)  # highest dock origin
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
# STORAGE SHELVES WITH GATE MECHANISM
# ---------------------------------------------------------------
# Each shelf is built in a local coordinate system that follows the shelf plate:
#   lx = 0..inner_w (width), ly = 0 front .. shelf_len rear (elevator),
#   lz = 0 at the shelf plate's surface. Then everything is tilted SLOPE_DEG.
#
# Gate mechanism per row (passive, no motor per row):
#   - A rocker arm under the shelf pivots about a shaft.
#   - Rear: a stop (post + spring-loaded flap) holds the queue.
#   - Front: a pin that sits lowered in the rest position.
#   - A side arm with a roller protrudes GATE_TAB mm into the elevator shaft.
#   When the elevator carriage pushes the roller down, the rear stop is
#   lowered and the front pin is raised between avocado 1 and 2: only one
#   avocado is released.
#   The flap on the rear stop can fold inwards, so the pusher can
#   push a new avocado into the row without opening the gate (LIFO).
#   Stop and pin heights follow the fruit's resting bottom (floor_z), which
#   depends on ROW_PROFILE.

rows_w = n_rows * row_pitch
x_off = WALL + (inner_w - rows_w) / 2
a_sl = -slope  # angle for local -> global

y_rear = stop_face + 2  # rear stop flap (centre)
y_front = stop_face - along_nom  # front pin (where avocado 1 and 2 touch)
y_piv = (y_rear + y_front) / 2
d_arm = (y_rear - y_front) / 2
z_piv = -SHELF_T - 14
phi0 = math.asin(GATE_TRAVEL / (2 * d_arm))  # ± rocker angle
L_tab = shelf_len + GATE_TAB - y_piv  # shaft -> roller
rear_len = floor_z + GATE_FLAP_H - (z_piv + d_arm * math.sin(phi0))
front_len = floor_z - 2 - (z_piv - d_arm * math.sin(phi0))
TAB_X = row_pitch / 2 - 12  # roller's lateral offset from row centre
shaft_half = row_pitch / 2 - 4  # rocker shaft half-length


def to_world(base, lx, ly, lz):
    """Local shelf point -> global coordinates."""
    return (
        WALL + lx,
        shelf_y0 + ly * math.cos(a_sl) - lz * math.sin(a_sl),
        base + ly * math.sin(a_sl) + lz * math.cos(a_sl),
    )


def row_cx(r):
    return x_off - WALL + r * row_pitch + row_pitch / 2


def tab_roller_z(phi):
    """Roller centre height (local) for a given rocker angle."""
    return z_piv + L_tab * math.sin(phi)


def gate_cutouts(cxl):
    """Openings for the rear stop and front pin (plate and tray)."""
    return [
        box(
            cxl - GATE_FLAP_W / 2 - 2, y_rear - 5, -SHELF_T - 1, GATE_FLAP_W + 4, 10, 60
        ),
        Pos(cxl, y_front, 0) * Cylinder(GATE_PIN_D / 2 + 2, 2 * SHELF_T + 2),
    ]


def row_floor(cxl):
    """What the fruit rests on in one row, besides the shelf plate."""
    if ROW_PROFILE == "v":
        w = row_inner / 2 / math.cos(math.radians(V_ANGLE))
        out = []
        for sgn in (-1, 1):
            flank = box(0 if sgn > 0 else -w, 0, -TRAY_T, w, shelf_len, TRAY_T)
            flank = Pos(cxl, 0, TRAY_T) * Rot(0, -sgn * V_ANGLE, 0) * flank
            for cut in gate_cutouts(cxl):
                flank -= cut
            out.append((flank, "Row tray (V-channel, lift-out)", "#e0f2f1"))
        return out
    if ROW_PROFILE == "rails":
        return [
            (
                Pos(cxl + sgn * RAIL_GAP / 2, shelf_len / 2, RAIL_D / 2)
                * Rot(90, 0, 0)
                * Cylinder(RAIL_D / 2, shelf_len),
                "Row rail",
                "#e0f2f1",
            )
            for sgn in (-1, 1)
        ]
    return []


def gate(cxl, released=False, flap_folded=False):
    """Gate mechanism for one row, in the shelf's local system."""
    phi = -phi0 if released else phi0
    out = []
    # Rocker arm (rotates about the shaft)
    rocker = [
        (Rot(0, 90, 0) * Cylinder(4, 2 * shaft_half), "Gate – shaft", "#9e9e9e"),
        (box(-4, -d_arm - 8, -3, 8, 2 * d_arm + 16, 6), "Gate – rocker arm", "#546e7a"),
        (box(TAB_X - 4, -6, -3, 8, L_tab + 6, 6), "Gate – side arm", "#546e7a"),
        (
            Pos(TAB_X, L_tab, 0) * Rot(0, 90, 0) * Cylinder(5, 12),
            "Gate – roller",
            "#ffca28",
        ),
    ]
    T = Pos(cxl, y_piv, z_piv) * Rot(math.degrees(phi), 0, 0)
    out += [(T * shp, lab, col) for shp, lab, col in rocker]

    # Rear stop: post + flap
    z_r = z_piv + d_arm * math.sin(phi)
    top_r = z_r + rear_len
    hinge_z = top_r - GATE_FLAP_H - 1
    out.append(
        (
            box(cxl - 10, y_rear - 2.5, z_r, 20, 5, hinge_z - z_r),
            "Gate – rear post",
            "#c0392b",
        )
    )
    out.append(
        (
            Pos(cxl, y_rear, hinge_z) * Rot(0, 90, 0) * Cylinder(3, GATE_FLAP_W),
            "Gate – flap hinge",
            "#7f8c8d",
        )
    )
    flap = box(-GATE_FLAP_W / 2, -2, 0, GATE_FLAP_W, 4, GATE_FLAP_H + 1)
    fold = 80 if flap_folded else 0
    out.append(
        (
            Pos(cxl, y_rear, hinge_z) * Rot(fold, 0, 0) * flap,
            "Gate – flap (one-way)",
            "#e74c3c",
        )
    )

    # Front pin
    z_f = z_piv - d_arm * math.sin(phi)
    pin_r = GATE_PIN_D / 2
    out.append(
        (
            Pos(cxl, y_front, z_f)
            * Cylinder(pin_r, front_len, align=(Align.CENTER, Align.CENTER, Align.MIN)),
            "Gate – front pin",
            "#c0392b",
        )
    )
    out.append(
        (
            Pos(cxl, y_front, z_f + front_len) * Sphere(pin_r),
            "Gate – pin tip",
            "#c0392b",
        )
    )

    # Fixed brackets under the shelf
    for sx in (-shaft_half, TAB_X - 14):
        out.append(
            (
                box(cxl + sx, y_piv - 8, z_piv - 8, 4, 16, -SHELF_T - (z_piv - 8)),
                "Gate – bracket",
                "#78909c",
            )
        )
    return out


docked = ELEV_MODE in ("fetch", "insert")
dock_origin = None

for s in range(n_shelves):
    zc = zone_color(s, n_shelves)
    temp = ZONE_TEMPS[s] if s < len(ZONE_TEMPS) else "?"
    base = z + drop  # highest point (front)
    T = Pos(WALL, shelf_y0, base) * Rot(-SLOPE_DEG, 0, 0)
    local = []

    # Shelf plate with cut-outs for the stops
    plate = box(0, 0, -SHELF_T, inner_w, shelf_len, SHELF_T)
    for r in range(n_rows):
        for cut in gate_cutouts(row_cx(r)):
            plate -= cut
    local.append((plate, f"Shelf {s + 1} ({temp} °C)", zc))

    # Row dividers
    for r in range(n_rows + 1):
        rx = x_off - WALL + r * row_pitch - DIVIDER_T / 2
        local.append(
            (box(rx, 0, 0, DIVIDER_T, shelf_len, DIVIDER_H), "Row divider", "#eeeeee")
        )

    # Hold-down: one rail per row, carried by cross bars fixed to the side walls
    if HOLD_DOWN:
        for fy in (0.2, 0.8):
            local.append(
                (
                    box(0, fy * shelf_len - 6, hold_z, inner_w, 12, HOLD_T),
                    "Hold-down cross bar",
                    "#b0bec5",
                )
            )

    # Row floors, gates + avocados
    n_fill = round(per_row_nom * FILL)
    for r in range(n_rows):
        cxl = row_cx(r)
        local += row_floor(cxl)
        if HOLD_DOWN:
            local.append(
                (
                    box(cxl - 6, 0, hold_z, 12, shelf_len, HOLD_T),
                    "Hold-down rail",
                    "#b0bec5",
                )
            )
        is_dock = docked and s == DOCK_SHELF - 1 and r == DOCK_ROW - 1
        local += gate(
            cxl,
            released=is_dock and ELEV_MODE == "fetch",
            flap_folded=is_dock and ELEV_MODE == "insert",
        )
        col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        is_load = s == 0 and r == load_row - 1
        if is_load:
            col = "#7d8a5c"  # not yet scanned
        if is_dock:
            dock_origin = to_world(base, cxl, shelf_len, 0)
        k0 = 1 if is_dock and ELEV_MODE == "fetch" else 0  # avocado 1 is in the cradle
        shift = queue_shift if is_dock and ELEV_MODE == "insert" else 0
        for k in range(k0, n_fill):
            cy = stop_face - slot_len / 2 - k * slot_len - shift
            lab = "Avocado (loaded, not scanned)" if is_load else "Avocado"
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

    z += level_pitch

# ---------------------------------------------------------------
# CUSTOMER SECTION ON TOP
# ---------------------------------------------------------------
sec_w = inner_w / len(TOP_SECTIONS)
top_colors = ["#2f3a1c", "#557a2b", "#86b83e"]

for i, name in enumerate(TOP_SECTIONS):
    sx = WALL + i * sec_w
    tray = box(0, 0, -12, sec_w - 6, top_len, 12)
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
# ELEVATOR (X-Z gantry at the rear) WITH DETAILED CRADLE
# ---------------------------------------------------------------
# The cradle is built in its own system (u, v, w) with origin at the shelf's
# rear edge (row centre, surface) when the carriage is docked:
#   u = sideways (X), v = backwards into the shaft (Y), w = up (Z)
#
# Fetch (ELEV_MODE = "fetch"):
#   1. The carriage positions itself ~27 mm above dock height at the correct row.
#   2. The tongue is pushed forward over the gate's roller (small solenoid).
#   3. The carriage lowers: the tongue presses the roller down -> the gate opens,
#      avocado 1 rolls over into the cradle, the pin holds the rest of the queue.
#   4. The carriage rises, the tongue retracts, the spring closes the gate.
# Insert (ELEV_MODE = "insert"):
#   The gate is closed. The pusher pushes the avocado past the spring-loaded
#   flap and pushes the whole queue one step up the slope. The flap snaps
#   up behind the avocado and holds it in place.
#   The same pusher also delivers to the customer sections on top.
# Scan (ELEV_MODE = "scan"):
#   After fetching an avocado from the loading row on the bottom shelf, the
#   carriage moves to SCAN_X and rises to the scan station. The camera looks
#   straight down into the cradle; the pusher's motor and rail sit under the
#   cradle so nothing blocks the view. The avocado is then placed on the shelf
#   for its ripeness (or directly in the customer section).
#
# The cradle pocket (cr_v0 .. push_rest) holds the largest fruit completely
# behind the gate rollers, so a fruit in the cradle never hits the rollers of
# other shelves while the carriage travels.

if ELEV_MODE == "scan":
    dock_origin = (scan_x, shelf_y1, z_scan)
if dock_origin is None:  # "free" or invalid dock position
    dock_origin = (
        x_travel0 + ELEV_X * (x_travel1 - x_travel0),
        shelf_y1,
        z_travel0 + ELEV_Z * (z_travel1 - z_travel0),
    )
ox, oy, oz = dock_origin
C = Pos(ox, oy, oz)


def cbox(u, v, w, du, dv, dw):
    return C * box(u, v, w, du, dv, dw)


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

# Cradle: bottom and side walls. The bottom has a slot for the pusher arm,
# offset from the row centre so the arm clears the gate's rear post and flap
# when it pushes into a row.
cr_depth = cr_v1 - cr_v0
cr_w0 = cradle_floor - CR_FLOOR_T  # underside of the cradle floor
bottom = cbox(-cr_w / 2, cr_v0, cr_w0, cr_w, cr_depth, CR_FLOOR_T) - cbox(
    PUSH_U - 5, cr_v0 - 1, cr_w0 - 1, 10, cr_depth + 2, CR_FLOOR_T + 2
)
add(bottom, "Cradle – bottom", "#ffb300")
for sgn in (-1, 1):
    u0 = cr_w / 2 if sgn > 0 else -cr_w / 2 - 4
    add(cbox(u0, cr_v0, cr_w0, 4, cr_depth, CR_WALL_H), "Cradle – side wall", "#ffa000")
add(
    cbox(-cr_w / 2 + 2, push_rest - 16, cradle_floor + 16, 8, 14, 10),
    "Sensor: avocado in cradle",
    "#212121",
)

# Tongue + solenoid that operates the gate's roller
tongue_top = tab_roller_z(-phi0) + 5  # top of the roller when the gate is open
if ELEV_MODE == "fetch":
    tv0 = GATE_TAB - 8  # tongue extended, above the roller
else:
    tv0 = cr_v0 + 8  # tongue retracted
add(cbox(TAB_X - 7, tv0, tongue_top, 14, 30, 5), "Tongue", "#e65100")
add(
    cbox(TAB_X - 12, cr_v0 + 14, tongue_top - 4, 24, 40, cr_w0 - (tongue_top - 4)),
    "Tongue solenoid",
    "#424242",
)

# Pusher: belt-driven linear slide under the cradle, the arm comes up
# through the slot in the cradle bottom (keeps the view from above clear)
add(
    cbox(-push_w / 2, push_face, cradle_floor + 10, push_w, 5, 56),
    "Pusher plate",
    "#f4511e",
)
add(
    cbox(PUSH_U - 4, push_face + 5, cradle_floor - 8, 8, 6, 28), "Pusher arm", "#bf360c"
)
add(
    cbox(PUSH_U - 8, push_face + 2, cradle_floor - 14, 16, 14, 6),
    "Pusher – slide",
    "#90a4ae",
)
add(
    cbox(PUSH_U - 10, cr_v0, cradle_floor - 24, 20, cr_depth, 10),
    "Pusher linear rail (belt)",
    "#546e7a",
)
add(
    cbox(PUSH_U - 14, cr_v1 - 32, cradle_floor - 48, 28, 32, 24),
    "Pusher motor",
    "#37474f",
)

# Avocado in the cradle (or on its way into the row)
if ELEV_MODE == "insert":
    avo_pos = (0, push_face - along_nom / 2, floor_z + AVO_D / 2)
else:
    avo_pos = cradle_avo
add(C * Pos(*avo_pos) * avocado_shape(), "Avocado", "#6b9a33")

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
        + (" + hold-down" if HOLD_DOWN else "")
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
    pin_ok = pin_shift <= pin_tol
    print(
        f" Front pin tolerance:       ±{pin_tol:.1f} mm, size range moves the"
        f" contact ±{pin_shift:.0f} mm"
    )
    if not pin_ok:
        print("   ! the front pin will hit fruit that is not close to nominal size")
    if ROW_PROFILE == "rails" and RAIL_GAP - RAIL_D < GATE_FLAP_W + 4:
        print("   ! rear stop flap does not fit between the rails")
    print("=" * 56)
    print(" GATE AND ELEVATOR")
    print("=" * 56)
    print(f" Gate rocker angle:         ±{math.degrees(phi0):.1f}°")
    print(
        f" Dock lowering (tongue):    {tab_roller_z(phi0) - tab_roller_z(-phi0):.0f} mm"
    )
    print(f" Pusher stroke into row:    {push_rest - push_end:.0f} mm")
    print(f" Cradle pocket (W x D):     {cr_w:.0f} x {push_rest - cr_v0:.0f} mm")
    print(f" Elevator shaft depth:      {elev_depth:.0f} mm")
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
    print(f" Level height (w/ slope):   {level_pitch:.0f} mm")
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
        print(f" Loading row buffer:        {load_cap}")
    if n_shelves < N_SHELVES:
        need = N_SHELVES * level_pitch + drop + BASE_H + TOP_H
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
