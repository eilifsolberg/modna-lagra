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

# Avocado (Hass, medium)
AVO_D = 70  # diameter
AVO_L = 100  # length
AVO_ROLLS = True  # True: long axis across the row (rolls)
# False: long axis along the row (slides)
CLEAR = 4  # clearance around avocado

# Vertical layout
BASE_H = 120  # technical compartment (cooling/heating, electronics)
TOP_H = 130  # customer section on top
LOAD_LEVEL = True  # bottom level = loading chute + scanner

# Shelves
N_SHELVES = 6  # desired number (the script checks whether it fits)
SHELF_T = 20  # shelf thickness (insulated divider between zones)
SLOPE_DEG = 3.0  # downward slope towards the elevator
FRONT_CLEAR = 10  # gap between shelf and front wall
GATE_SPACE = 25  # space under the shelf for the gate mechanism
FILL = 0.7  # fraction of slots shown filled

# Gate (escapement) at the rear end of each row
GATE_TRAVEL = 19  # vertical travel of the stops (mm)
GATE_FLAP_H = 17  # height of the rear stop above the shelf plate
GATE_TAB = 12  # how far the rocker arm's roller protrudes into the elevator shaft

# Elevator (rear side)
ELEV_DEPTH = 150  # depth of the elevator shaft
ELEV_MODE = "fetch"  # "fetch":  the carriage is docked and releases one avocado
# "insert": the pusher pushes an avocado into the row
# "free":   the carriage is free-standing at ELEV_X / ELEV_Z
DOCK_SHELF = 2  # shelf the carriage is docked against (1 = lowest storage shelf)
DOCK_ROW = 2  # row the carriage is docked against (1 = left)
ELEV_X = 0.35  # only for "free" (0..1)
ELEV_Z = 0.55  # only for "free" (0..1)

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
shelf_y0 = WALL + FRONT_CLEAR
shelf_y1 = D - WALL - ELEV_DEPTH
shelf_len = shelf_y1 - shelf_y0
slope = math.radians(SLOPE_DEG)
drop = shelf_len * math.tan(slope)

row_pitch = (AVO_L if AVO_ROLLS else AVO_D) + CLEAR
slot_len = (AVO_D if AVO_ROLLS else AVO_L) + 2
n_rows = int(inner_w // row_pitch)
per_row = int(shelf_len // slot_len)

level_pitch = AVO_D + CLEAR + SHELF_T + GATE_SPACE
storage_z0 = BASE_H
storage_z1 = H - TOP_H
n_levels_fit = int((storage_z1 - storage_z0 - drop) // level_pitch)
n_storage_fit = n_levels_fit - (1 if LOAD_LEVEL else 0)
n_shelves = min(N_SHELVES, n_storage_fit)

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


def avocado(x, y, z, color, along_x=True):
    a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
    if not along_x:
        a = Rot(0, 0, 90) * a
    return add(Pos(x, y, z) * a, "Avocado", color)


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
    add(box(WALL, D - WALL, 0, inner_w, WALL, H - TOP_H), "Rear wall", "#cfcfcf")
    add(box(WALL, 0, 0, inner_w, WALL, BASE_H), "Front plinth", "#bdbdbd")
add(
    box(WALL, WALL, 0, inner_w, D - 2 * WALL, BASE_H - 5),
    "Technical compartment (cooling/heating, control)",
    "#7a7a7a",
)

# ---------------------------------------------------------------
# LOADING LEVEL: chute from the front -> scanner -> elevator
# ---------------------------------------------------------------
z = storage_z0
if LOAD_LEVEL:
    chute_w = row_pitch * 1.3
    chute_len = shelf_y1 - WALL
    chute_top = z + chute_len * math.tan(slope) + 5
    chute = box(0, 0, -SHELF_T / 2, chute_w, chute_len, SHELF_T / 2)
    chute = Pos(W / 2 - chute_w / 2, WALL, chute_top) * Rot(-SLOPE_DEG, 0, 0) * chute
    add(chute, "Loading chute", "#b0b0b0")
    # loading opening in the front (frame)
    add(
        box(W / 2 - 80, 0, z, 160, WALL, 10), "Loading opening – lower frame", "#444444"
    )
    add(
        box(W / 2 - 80, 0, z + AVO_D + 30, 160, WALL, 10),
        "Loading opening – upper frame",
        "#444444",
    )
    # multispectral camera above the chute, near the elevator
    cam_y = shelf_y1 - 190
    cam_z = z + AVO_D + 12  # placed directly below shelf 1
    add(box(W / 2 - 45, cam_y, cam_z, 90, 70, 16), "Multispectral camera", "#1f1f1f")
    add(Pos(W / 2, cam_y + 35, cam_z - 3) * Cylinder(15, 6), "Camera lens", "#3050a0")
    avocado(
        W / 2,
        cam_y + 35,
        chute_top - (cam_y + 35 - WALL) * math.tan(slope) + AVO_D / 2,
        "#557a2b",
        along_x=AVO_ROLLS,
    )
    z += level_pitch

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

rows_w = n_rows * row_pitch
x_off = WALL + (inner_w - rows_w) / 2
a_sl = -slope  # angle for local -> global

y_rear = shelf_len - 4  # rear stop (centre)
y_front = shelf_len - 6 - slot_len  # front pin (between avocado 1 and 2)
y_piv = (y_rear + y_front) / 2
d_arm = (y_rear - y_front) / 2
z_piv = -SHELF_T - 14
phi0 = math.asin(GATE_TRAVEL / (2 * d_arm))  # ± rocker angle
L_tab = shelf_len + GATE_TAB - y_piv  # shaft -> roller
rear_len = GATE_FLAP_H - (z_piv + d_arm * math.sin(phi0))
front_len = -2 - (z_piv - d_arm * math.sin(phi0))
TAB_X = row_pitch / 2 - 12  # roller's lateral offset from row centre


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


def gate(cxl, released=False, flap_folded=False):
    """Gate mechanism for one row, in the shelf's local system."""
    phi = -phi0 if released else phi0
    out = []
    # Rocker arm (rotates about the shaft)
    rocker = [
        (Rot(0, 90, 0) * Cylinder(4, 100), "Gate – shaft", "#9e9e9e"),
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
            Pos(cxl, y_rear, hinge_z) * Rot(0, 90, 0) * Cylinder(3, 36),
            "Gate – flap hinge",
            "#7f8c8d",
        )
    )
    flap = box(-18, -2, 0, 36, 4, GATE_FLAP_H + 1)
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
    out.append(
        (
            Pos(cxl, y_front, z_f)
            * Cylinder(4, front_len, align=(Align.CENTER, Align.CENTER, Align.MIN)),
            "Gate – front pin",
            "#c0392b",
        )
    )
    out.append(
        (Pos(cxl, y_front, z_f + front_len) * Sphere(4), "Gate – pin tip", "#c0392b")
    )

    # Fixed brackets under the shelf
    for sx in (-50, 26):
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
shelf_bases = []

for s in range(n_shelves):
    zc = zone_color(s, n_shelves)
    temp = ZONE_TEMPS[s] if s < len(ZONE_TEMPS) else "?"
    base = z + drop  # highest point (front)
    shelf_bases.append(base)
    T = Pos(WALL, shelf_y0, base) * Rot(-SLOPE_DEG, 0, 0)
    local = []

    # Shelf plate with cut-outs for the stops
    plate = box(0, 0, -SHELF_T, inner_w, shelf_len, SHELF_T)
    for r in range(n_rows):
        cxl = row_cx(r)
        plate -= box(cxl - 20, y_rear - 5, -SHELF_T - 1, 40, 10, SHELF_T + 2)
        plate -= Pos(cxl, y_front, -SHELF_T / 2) * Cylinder(6, SHELF_T + 2)
    local.append((plate, f"Shelf {s + 1} ({temp} °C)", zc))

    # Row dividers
    for r in range(n_rows + 1):
        rx = x_off - WALL + r * row_pitch - 2
        local.append((box(rx, 0, 0, 4, shelf_len, 35), "Row divider", "#eeeeee"))

    # Gates + avocados
    n_fill = round(per_row * FILL)
    for r in range(n_rows):
        cxl = row_cx(r)
        is_dock = docked and s == DOCK_SHELF - 1 and r == DOCK_ROW - 1
        local += gate(
            cxl,
            released=is_dock and ELEV_MODE == "fetch",
            flap_folded=is_dock and ELEV_MODE == "insert",
        )
        col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        k0, shift = 0, 0
        if is_dock and ELEV_MODE == "fetch":
            k0 = 1  # avocado 1 has rolled over into the cradle
            dock_origin = to_world(base, cxl, shelf_len, 0)
        if is_dock and ELEV_MODE == "insert":
            shift = 40  # the queue has been pushed one step forward
            dock_origin = to_world(base, cxl, shelf_len, 0)
        for k in range(k0, n_fill):
            cy = shelf_len - 6 - slot_len / 2 - k * slot_len - shift
            a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
            if not AVO_ROLLS:
                a = Rot(0, 0, 90) * a
            local.append((Pos(cxl, cy, AVO_D / 2 + 1) * a, "Avocado", col))

    for shp, lab, col in local:
        add(T * shp, lab, col)

    z += level_pitch

# ---------------------------------------------------------------
# CUSTOMER SECTION ON TOP
# ---------------------------------------------------------------
top_z = H - TOP_H + 20
sec_w = inner_w / len(TOP_SECTIONS)
top_len = shelf_y1 - WALL - 5
top_slope = math.radians(TOP_SLOPE_DEG)
top_drop = top_len * math.tan(top_slope)
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
            avocado(ax, ay, az, top_colors[i], along_x=False)

if SHOW_HOUSING:
    # rear top plate (covers the elevator's top station)
    add(
        box(WALL, D - WALL - ELEV_DEPTH, H - 20, inner_w, ELEV_DEPTH + WALL, 20),
        "Top plate above elevator",
        "#cfcfcf",
    )

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

x_travel0 = WALL + 30
x_travel1 = W - WALL - 30
z_travel0 = storage_z0
z_travel1 = H - 40
SHAFT = D - WALL - shelf_y1  # shaft depth in the v direction

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
for zr in (z_travel0 - 30, z_travel1):
    add(box(WALL, D - WALL - 24, zr, inner_w, 24, 18), "X rail", "#607d8b")
col_z0, col_z1 = z_travel0 - 12, z_travel1
add(
    box(ox - 20, oy + 126, col_z0, 40, SHAFT - 126, col_z1 - col_z0),
    "Z actuator (column)",
    "#455a64",
)
add(box(ox - 8, oy + 120, col_z0, 16, 6, col_z1 - col_z0), "Z linear rail", "#b0bec5")

# Carriage
add(cbox(-55, 92, -55, 110, 16, 155), "Carriage plate", "#37474f")
for w0 in (-45, 55):
    add(cbox(-22, 108, w0, 44, 12, 30), "Bearing block", "#263238")

# Cradle: bottom and side walls (inner width > avocado length)
CR_W = (AVO_L if AVO_ROLLS else AVO_D) + 9
add(cbox(-CR_W / 2, 16, -13, CR_W, 76, 5), "Cradle – bottom", "#ffb300")
for sgn in (-1, 1):
    u0 = CR_W / 2 if sgn > 0 else -CR_W / 2 - 4
    add(cbox(u0, 16, -13, 4, 76, 50), "Cradle – side wall", "#ffa000")
add(cbox(-CR_W / 2 + 2, 60, 8, 8, 14, 10), "Sensor: avocado in cradle", "#212121")

# Tongue + solenoid that operates the gate's roller
tongue_top = tab_roller_z(-phi0) + 5  # top of the roller when the gate is open
if ELEV_MODE == "fetch":
    tv0 = GATE_TAB - 8  # tongue extended, above the roller
else:
    tv0 = 24  # tongue retracted
add(cbox(TAB_X - 7, tv0, tongue_top, 14, 30, 5), "Tongue", "#e65100")
add(
    cbox(TAB_X - 12, 30, tongue_top - 4, 24, 40, -13 - (tongue_top - 4)),
    "Tongue solenoid",
    "#424242",
)

# Pusher driven by a small linear actuator above the cradle
push_face = 76 if ELEV_MODE != "insert" else 30
add(cbox(-40, push_face, 2, 80, 5, 56), "Pusher plate", "#f4511e")
add(cbox(-6, push_face + 5, 50, 12, 8, 30), "Pusher arm", "#bf360c")
add(cbox(-14, push_face + 2, 78, 28, 16, 4), "Pusher – slide", "#90a4ae")
add(cbox(-18, 20, 82, 36, 90, 18), "Pusher linear actuator", "#546e7a")
add(cbox(-18, 104, 82, 36, 6, 18), "Actuator mount", "#37474f")

# Avocado in the cradle
a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
if not AVO_ROLLS:
    a = Rot(0, 0, 90) * a
if ELEV_MODE == "insert":
    add(C * Pos(0, push_face - AVO_D / 2, AVO_D / 2 + 4) * a, "Avocado", "#6b9a33")
else:
    add(C * Pos(0, push_face - AVO_D / 2, -8 + AVO_D / 2) * a, "Avocado", "#6b9a33")

# ---------------------------------------------------------------
# EXPORT + REPORT
# ---------------------------------------------------------------
model = Compound(children=parts, label="Avocado Manager")

if __name__ == "__main__":
    export_step(model, "avocado_manager.step")

    print("=" * 56)
    print(" GATE AND ELEVATOR")
    print("=" * 56)
    print(f" Gate rocker angle:         ±{math.degrees(phi0):.1f}°")
    print(
        f" Dock lowering (tongue):    {tab_roller_z(phi0) - tab_roller_z(-phi0):.0f} mm"
    )
    print(f" Pusher stroke into row:    {76 - (-6):.0f} mm")
    print(f" Cradle inner width:        {CR_W:.0f} mm")
    print("=" * 56)
    print(" CAPACITY REPORT")
    print("=" * 56)
    print(f" Shelf area (W x D):        {inner_w:.0f} x {shelf_len:.0f} mm")
    print(f" Rows per shelf:            {n_rows}  (row width {row_pitch:.0f} mm)")
    print(f" Avocados per row:          {per_row}")
    print(f" Avocados per shelf:        {n_rows * per_row}")
    print(f" Level height (w/ slope):   {level_pitch:.0f} mm")
    print(
        f" Levels that fit:           {n_levels_fit}"
        + ("  (1 used for loading)" if LOAD_LEVEL else "")
    )
    print(f" Storage shelves in model:  {n_shelves}  (desired {N_SHELVES})")
    print(f" Total storage capacity:    {n_shelves * n_rows * per_row}")
    if n_shelves < N_SHELVES:
        need = (
            (N_SHELVES + (1 if LOAD_LEVEL else 0)) * level_pitch + drop + BASE_H + TOP_H
        )
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
