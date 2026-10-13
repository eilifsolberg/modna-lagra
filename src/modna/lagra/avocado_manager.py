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

import functools
import itertools
import math

import rigid_chain as rc  # the fork's chain: links, loads, drive
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

# Avocado (Hass), lying with its long axis along the rods (front to back).
# The nominal size is what is drawn. Lanes, level height, lip, fork and cradle
# are sized for the whole range, and the report checks capacity and the lip /
# fork function across it.
AVO_D = 65  # nominal diameter
AVO_L = 95  # nominal length
AVO_D_RANGE = (62, 78)  # min/max diameter (rough Hass range - measure your supply)
AVO_L_RANGE = (85, 115)  # min/max length
CLEAR = 4  # clearance around the largest avocado

# Vertical layout
BASE_H = 120  # technical compartment (cooling/heating, electronics)
TOP_H = 130  # customer section on top

# Shelves: a bed of parallel steel rods running front to back, with no row
# dividers. A fruit settles centred over the gap between two rods, so it can
# lie over any gap: lanes are chosen by the software to suit each fruit's
# width, and only their centres are tied to the rod pitch. The fork's tines run
# in the gaps just outside the two rods a fruit rests on.
N_SHELVES = 6  # desired number of levels, incl. the loading drawer (the script
# checks whether they fit)
ROD_D = 7  # rod diameter (stainless)
ROD_PITCH = 15  # centre distance between rods
ROD_EDGE = 10  # outermost rods at least this far from the side walls
XBAR_H = 8  # rear crossbar under the rods' rear ends (flat bar, wall to wall;
# a narrow V-seat under each rod, nothing between them rises above the rods'
# underside, so the tines and the chain's keel pass over it)
XBAR_D = 20  # rear crossbar depth (Y)
STOP_T = 10  # front block: holds the rods' front ends and is every lane's stop
STOP_H = 45  # front block height above the rod tops
FRONT_CLEAR = 10  # gap between shelf and front wall
LANE_GAP = 5  # min gap between fruit in neighbouring lanes
PLACE_GAP = 5  # gap the fork leaves between stored fruit in a lane
FILL = 0.7  # fraction of slots shown filled

# Loading drawer: the top storage level, just under the customer section, is a
# drawer that staff pull out and fill from above. It has the same rod bed, but
# fixed lanes with dividers to guide people, and every lane is a loading lane:
# fruit are pushed back against a fixed rear lip (two posts with a notch for
# the fork), and the fork lifts them over it. The drawer front closing against
# the housing positions the lanes for the fork; the elevator only fetches from
# the drawer when a sensor reports it closed.
LOAD_DRAWER = True  # False: no loading level (all levels are storage)
DIVIDER_T = 4  # lane divider (stands in a rod gap, on the crossbars)
DIVIDER_H = 35  # divider height above the rod tops
LIP_H = 8  # lip top above the resting (nominal) fruit's bottom
LIP_BACK = 3  # lip material behind the rear point of the fruit resting against it
RUNNER_W = 8  # fixed runners on the side walls that the drawer slides on
RUNNER_H = 10
DRAWER_KEEP = 60  # drawer length that stays on the runners when fully open
DRAWER_OPEN = 0.6  # snapshot: how far the drawer is pulled out (0..1)

# Fork on the carriage: two steel tines + pusher face, driven by a rigid chain
# (it can only bend upwards, so it holds the fruit out straight, and turns up
# into a vertical magazine behind the cradle), so it reaches the front of a
# lane from a shallow carriage. It is the only actuator on the carriage; the
# elevator's Z axis does the lifting. The tines sit in the rod gaps either side
# of the fruit's two rods, the chain body rides above the rods and its keel
# runs in the gap under the fruit. The chain itself (links, keel, pins,
# sprocket, loads) is in rigid_chain.py.
TINE_W = 4  # tine width (steel flat bar)
TINE_H = 3.5  # tine height: low enough to stay above the rods' underside
TINE_CLEAR = 3  # clearance tine <-> fruit when sliding under it
XBAR_CLEAR = 1  # min gap between the fork's underside and the rear crossbar
CHAIN_CLEAR = 1.5  # min gap between the chain body and the rod tops, fork low
LIFT_CLEAR = 3  # a carried fruit clears the rods (and loading lip) by this
TIP_MARGIN = 8  # the tip stays this far ahead of a fruit's centre and behind the next
PUSHER_T = 6  # pusher face thickness
PUSHER_H = 30  # pusher face height above the tines
HEAD_L = 10  # chain adapter behind the pusher face, up to the first pin
HEAD_G = 40  # fork head (pusher + tines) mass, g

# Elevator (rear side). The shaft depth follows from the carriage stack below.
ELEV_MODE = "scan"  # "fetch":  the fork has lifted the rearmost avocado of a lane
# "insert": the fork carries an avocado to its place in the lane
# "scan":   an avocado is on the imaging module's rollers, the fork retracted
# "free":   the carriage is free-standing at ELEV_X / ELEV_Z
DOCK_SHELF = 2  # level the carriage is docked against (1 = lowest; the top
# level is the loading drawer)
DOCK_LANE = 2  # lane the carriage is docked against (1 = left)
ELEV_X = 0.35  # only for "free" (0..1)
ELEV_Z = 0.55  # only for "free" (0..1)
FORK_SNAPSHOT = 0.5  # only for "insert": how far towards its place the avocado is

# Carriage, front to back: cradle pocket, pusher face, drive sprocket, then
# the chain magazine - a straight vertical channel the chain stands up in.
# Two Z columns, one each side of the magazine, carry the carriage on side
# cheeks; the shaft depth follows from the sprocket and magazine.
CR_GAP = 5  # gap between the shelves' rear edge and the cradle
CR_WALL_H = 50  # cradle side wall height above the tines
MAG_CLEAR = 3  # clearance around the sprocket and magazine
MAG_WALL = 3  # magazine housing wall
CHEEK_T = 8  # carriage side cheeks
CHEEK_DOWN = 25  # cheeks reach this far below the cradle base
BEARING_T = 12  # Z carriage block (MGN12H-like): depth, width
BEARING_W = 27
Z_RAIL_T = 8
COLUMN_T = 24

# Imaging drawer: a level of its own directly under the loading drawer. The
# imaging module stands at its rear on the right (the rest is free), where
# the fork serves it like a lane (a short bed with the shelves' rod
# geometry). Staff pull the drawer out to clean, calibrate or swap the
# module. It is inside the closed body, so it is dark and at storage
# temperature. It rides on runners like the loading drawer.
IM_PLATE_T = 6  # imaging drawer's plate
MODULE_T = 4  # module base plate, bolted onto the drawer's plate
IM_DRAWER_OPEN = 0.0  # snapshot: how far the imaging drawer is pulled out (0..1)
RAIL_CLEAR = 15  # clearance between the carriage top and the upper X rail
ROOF_T = 20  # roof of the raised back (over the elevator shaft)

# Imaging module: two driven rollers (Ø ROD_D at ROD_PITCH, like the two rods
# a stored fruit rests on) spin the fruit about its long axis in front of a
# side camera on a motorised distance slide. There is no guard on the camera
# side; a matte background panel on the far side carries the reference tile.
# A narrow front stop keeps the spinning fruit from walking. The fruit's ends
# are only seen at a slant; an end camera can be added later.
BED_SPARE = 10  # bed length beyond the longest fruit
BED_STOP_H = 15  # front stop above the roller tops
STOP_W = 30  # front stop width (holds the rollers' bearings)
BG_UP = 20  # background panel reaches this far above the largest fruit
DRIVE_L = 15  # roller belt in front of the stop (its stepper is beside the bed)
SPIN_STOPS = 6  # rotation stops per fruit (all LED bands are imaged at each)
CAM_BODY = (60, 60, 25)  # camera + LED holder: width, height, depth (view axis)
CAM_DIST_RANGE = (110, 220)  # side camera's slide range, its centre to the fruit's
CAM_DIST = 140  # side camera distance in the snapshot (within CAM_DIST_RANGE)
SLIDE_END = 35  # slide's stepper, beyond the camera's farthest position

# Customer section on top
TOP_SECTIONS = ["Soft ripe", "Firm ripe", "Ripe in a few days"]
TOP_SLOPE_DEG = 6.0  # slopes towards the customer
TRAY_T = 12  # tray thickness

SHOW_HOUSING = True

# ---------------------------------------------------------------
# DERIVED QUANTITIES
# ---------------------------------------------------------------
inner_w = W - 2 * WALL

# Fruit size across the lane (its diameter) and along it (its length)
across_nom, along_nom = AVO_D, AVO_L
across_min, across_max = AVO_D_RANGE
along_min, along_max = AVO_L_RANGE
# Fruit sizes checked below, as (across, along, height): smallest, nominal, largest
SIZES = [
    (AVO_D_RANGE[0], AVO_L_RANGE[0], AVO_D_RANGE[0]),
    (AVO_D, AVO_L, AVO_D),
    (AVO_D_RANGE[1], AVO_L_RANGE[1], AVO_D_RANGE[1]),
]


@functools.cache
def rest_height(across, height):
    """Fruit centre above the rod tops. The fruit settles centred over a gap
    between two rods; its cross-section across the lane is treated as an
    ellipse (across x height). Bisection for the lowest centre where it clears
    every rod under it."""
    a, b, r = across / 2, height / 2, ROD_D / 2
    rods = [(k + 0.5) * ROD_PITCH for k in range(int(a // ROD_PITCH) + 1)]
    ts = [2 * math.pi * i / 720 for i in range(720)]

    def clears(z_c):  # rod centres at (±x, -r); symmetric, so +x only
        return all(
            math.hypot(a * math.cos(t) - x, z_c + b * math.sin(t) + r) >= r
            for x in rods
            for t in ts
        )

    lo, hi = b - ROD_D, b  # sunk to the rods' underside .. resting on their tops
    for _ in range(30):
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if clears(mid) else (mid, hi)
    return hi


def on_points(across, height, u):
    """Centre of the fruit's cross-section above two support points at ±u."""
    a, b = across / 2, height / 2
    return b * math.sqrt(1 - (u / a) ** 2)


rest_nom = rest_height(across_nom, AVO_D)  # nominal fruit centre above rod tops
floor_z = rest_nom - AVO_D / 2  # nominal fruit bottom (below the rod tops)
fruit_top_max = max(rest_height(ac, h) + h / 2 for ac, _, h in SIZES)

# Rod bed: rods across the inner width, centred. Lane centres are gap centres.
rod_gap = ROD_PITCH - ROD_D
n_rods = int((inner_w - 2 * ROD_EDGE) // ROD_PITCH) + 1
rod_x0 = (inner_w - (n_rods - 1) * ROD_PITCH) / 2
rod_xs = [rod_x0 + i * ROD_PITCH for i in range(n_rods)]
gap_xs = [x + ROD_PITCH / 2 for x in rod_xs[:-1]]
bed = ROD_D + XBAR_H  # rod tops -> underside of the rear crossbar

# Tines in the gaps just outside the fruit's two rods; the chain's keel in the
# gap between them. The notch in the loading lip and in the top trays' rear
# ends lets the whole fork head through.
TINE_GAP = 2 * ROD_PITCH  # centre distance between the two tines
tine_in = TINE_GAP / 2 - TINE_W / 2  # inner edge of a tine (the fruit rests on it)
notch = TINE_GAP / 2 + TINE_W / 2 + TINE_CLEAR  # half width of the notches
tine_side = (rod_gap - TINE_W) / 2  # sideways clearance tine <-> rod
keel_side = (rod_gap - rc.KEEL_W) / 2  # ... keel <-> rod

# Carriage width: the Z columns stand either side of the chain magazine, so a
# lane centre must be this far from a side wall for the elevator to reach it
mag_w = rc.pin_len + 2 * MAG_WALL  # magazine width, pin ends included
col_u = mag_w / 2 + MAG_CLEAR + BEARING_W / 2  # column centre, sideways
reach = col_u + COLUMN_T / 2


def lane_centres(across):
    """Lanes for fruit `across` wide, packed from the left: each on the first
    gap centre where the fruit clears the side wall and the previous lane's
    fruit by LANE_GAP, and the elevator reaches. (The machine does the same per
    fruit, with each lane as wide as the widest fruit in it.)"""
    out, left = [], CLEAR
    for x in gap_xs:
        fits = x - across / 2 >= left and x + across / 2 <= inner_w - CLEAR
        if fits and reach <= x <= inner_w - reach:
            out.append(x)
            left = x + across / 2 + LANE_GAP
    return out


lanes_nom = lane_centres(across_nom)
lanes_min = lane_centres(across_min)
lanes_max = lane_centres(across_max)

# Loading drawer lanes: fixed, for the largest fruit, with the dividers between
# them standing in rod gaps (so the lane pitch is an even number of pitches)
lane_inner = across_max + 2 * CLEAR  # drawer lane and cradle inner width
load_pitch_x = 2 * ROD_PITCH * math.ceil((lane_inner + DIVIDER_T) / (2 * ROD_PITCH))
load_half = (load_pitch_x - DIVIDER_T) / 2  # half the drawer lane's inner width
load_edge = max(reach, load_half)  # nearest drawer lane centre to a side wall
load_lanes = []
for x in gap_xs:
    next_free = not load_lanes or x - load_lanes[-1] > load_pitch_x - 0.1
    if next_free and load_edge <= x <= inner_w - load_edge:
        load_lanes.append(x)

# Loading lane lip
lip_top = floor_z + LIP_H  # above the rod tops


def lip_contact(across, along, height):
    """Fruit resting against the loading lane's lip, touching the posts' top
    inner edges. Returns (bulge, hold): its rear bulges `bulge` mm past the
    posts' front face into the notch, and climbing the lip takes a push of
    `hold` x its weight (pivoting about the edge)."""
    a, l, b = across / 2, along / 2, height / 2
    dz = max(rest_height(across, height) - lip_top, 0.0)  # contact below the centre
    ext = l * math.sqrt(max(0.0, 1 - (notch / a) ** 2 - (dz / b) ** 2))
    return l - ext, ext / dz if dz > 0 else math.inf


contacts = [lip_contact(*s) for s in SIZES]
bulge_nom = contacts[1][0]
lip_d = max(bg for bg, _ in contacts) + LIP_BACK  # depth of the lip posts
hold_min = min(h for _, h in contacts)

# Fork heights above the rod tops. Low: the tines slide in under a fruit.
# High: a fruit on the tines (it sags between them) clears the rods, and in
# the loading lane the lip posts outside the notch.
tine_top_low = (
    min(rest_height(ac, h) - on_points(ac, h, tine_in) for ac, _, h in SIZES)
    - TINE_CLEAR
)
tine_top_high = max(
    rest_height(ac, h) + LIFT_CLEAR - on_points(ac, h, tine_in) for ac, _, h in SIZES
)
tine_top_high_load = max(
    lip_top + LIFT_CLEAR + on_points(ac, h, notch) - on_points(ac, h, tine_in)
    for ac, _, h in SIZES
)
lift = tine_top_high - tine_top_low  # Z move of the elevator per fetch/insert
lift_load = tine_top_high_load - tine_top_low  # ... in the loading lane


def lifted_top(tine_top):
    """Top of the largest fruit on the tines, above the rod tops."""
    return max(tine_top + on_points(ac, h, tine_in) + h / 2 for ac, _, h in SIZES)


# Fork underside (tines and keel bottom) and the chain body's bottom face, fork
# low. The underside must clear the rear crossbar, the body the rod tops.
w_fork = tine_top_low - TINE_H
w_chain = w_fork + rc.KEEL_H
xbar_gap = w_fork + ROD_D  # fork underside above the crossbar
chain_gap = w_chain  # chain body above the rod tops
# the cradle base under the tines and keel is the chain's front support; a
# fruit on the tines must pass over it into the cradle
fruit_low = min(tine_top_low + on_points(ac, h, tine_in) - h / 2 for ac, _, h in SIZES)
base_gap = fruit_low - (w_fork - 1)

# Tine length. When fetching, the pusher face touches the fruit's rear (its
# size and place are known from the scan), so the tip must reach past the
# fruit's centre but stay behind the next fruit, for every fruit size.
tip_lo = along_max / 2 + TIP_MARGIN
tip_hi = along_min - TIP_MARGIN
tine_len = (tip_lo + tip_hi) / 2 if tip_lo <= tip_hi else tip_hi

# Carriage in v (backwards from the shelf's rear edge, v < 0 is inside the
# lane): cradle pocket for the largest fruit, pusher face, drive sprocket and
# the vertical magazine, with the Z guides beside it at the back
cr_v0 = CR_GAP  # cradle front
push_rest = cr_v0 + along_max + CLEAR  # pusher face at rest (back of the pocket)
cr_v1 = push_rest + PUSHER_T + MAG_CLEAR  # back of the cradle
v_drive = cr_v1 + rc.r_plate  # sprocket centre, clear of the pusher at rest
# the chain turns up round the sprocket (pins on the inside) and stands in a
# channel from mag_v0 to mag_v1 (outside of the walls), keels at the back
mag_v0 = v_drive + rc.r_sprocket - rc.PIN_E - rc.MAG_FIT - MAG_WALL
mag_v1 = mag_v0 + rc.chain_depth + 2 * (rc.MAG_FIT + MAG_WALL)
elev_depth = max(  # depth of the elevator shaft
    mag_v1 + MAG_CLEAR, cr_v1 + 2 * CHEEK_T + BEARING_T + Z_RAIL_T + COLUMN_T
)
v_col = elev_depth - COLUMN_T
v_zrail = v_col - Z_RAIL_T
v_bear = v_zrail - BEARING_T

# Shelves
shelf_y0 = WALL + FRONT_CLEAR
shelf_y1 = D - WALL - elev_depth
shelf_len = shelf_y1 - shelf_y0
stop_face = shelf_len - lip_d  # front face of the loading lane's lip posts
q_rear_load = stop_face + bulge_nom  # rear point of the fruit against the lip
v_deep = STOP_T + along_min + 2 - shelf_len  # pusher face at the frontmost fruit
fork_reach = push_rest - v_deep  # fork stroke, rest -> front of a lane


def per_lane(along):
    return int((shelf_len - STOP_T + PLACE_GAP) // (along + PLACE_GAP))


def per_load_lane(along):
    return int((q_rear_load - STOP_T) // along)


def queue(is_load, n):
    """Centres (ly) of n nominal fruit in a lane, front to rear. Stored fruit
    sit from the front stop with gaps; the loading lane is pushed in by hand
    against the rear lip."""
    if is_load:
        return [q_rear_load - along_nom / 2 - k * along_nom for k in range(n)][::-1]
    return [STOP_T + along_nom / 2 + k * (along_nom + PLACE_GAP) for k in range(n)]


per_lane_nom = per_lane(along_nom)

# Level height: rod tops to rod tops. A lifted fruit leaving a lane passes
# under the rear crossbar of the level above, so the whole bed counts.
level_pitch = max(fruit_top_max, lifted_top(tine_top_high)) + CLEAR + bed
# the loading drawer's level is taller if the lift over its lips needs it
load_pitch = max(
    level_pitch,
    lifted_top(tine_top_high_load) + CLEAR + bed if LOAD_DRAWER else 0,
)
storage_z0 = BASE_H
storage_z1 = H - TOP_H  # underside of the customer section at the front

# Customer section on top
top_z = H - TOP_H + 20
top_len = shelf_y1 - WALL - 5
top_slope = math.radians(TOP_SLOPE_DEG)
top_drop = top_len * math.tan(top_slope)


# Imaging drawer, on top of the storage levels (under the loading drawer).
# The module's bed runs from its front stop (v_stop, in the carriage's v at
# the scan dock) to the shelves' rear edge.
v_stop = -(along_max + BED_SPARE)
w_mod = max(  # module's height above the roller tops
    lifted_top(tine_top_high),  # a fruit carried in
    fruit_top_max + BG_UP,  # background panel
    rest_nom + CAM_BODY[0] / 2,  # side camera
)
# its level: plate underside -> underside of the next level's bed
im_h = IM_PLATE_T + MODULE_T + bed + w_mod + CLEAR
avail_h = storage_z1 - storage_z0 - im_h  # for the rod-bed levels
n_levels_fit = (
    1 + int((avail_h - load_pitch) // level_pitch) if avail_h >= load_pitch else 0
)
n_shelves = min(N_SHELVES, n_levels_fit)  # levels in the model, incl. the drawer
n_storage = n_shelves - 1 if LOAD_DRAWER and n_shelves else n_shelves
drawer_pull = min(max(DRAWER_OPEN, 0), 1) * (shelf_len - DRAWER_KEEP)
im_z0 = storage_z0 + n_storage * level_pitch  # imaging drawer plate underside
im_pull = min(max(IM_DRAWER_OPEN, 0), 1) * (shelf_len - DRAWER_KEEP)


# Elevator travel. The carriage is built around a dock origin at the shelf's
# rear edge (lane centre, rod tops): there the fork is low, `lift` higher it
# is high.
def fruit_on_tines(across, along, height, face):
    """(v, w) of a fruit's centre on the tines, rear against the pusher face."""
    return face - along / 2, tine_top_low + on_points(across, height, tine_in)


cr_w = lane_inner  # cradle inner width
x_travel0 = WALL + 30
x_travel1 = W - WALL - 30
z_travel0 = storage_z0 + bed  # lowest dock origin (the bottom level's rod tops)
z_deliver = top_z + top_len * math.sin(top_slope)  # rear end of the top trays

# Rigid chain on the carriage. It runs from the fork head, its keel sliding on
# the cradle base, back to the drive sprocket, with its body above the rods.
# There it turns up a quarter round the sprocket and stands in the vertical
# magazine, which holds the full stroke.
w_drive = w_chain + rc.w_pin + rc.r_sprocket  # sprocket centre
chain_len = fork_reach + rc.PITCH  # stroke + a spare link
mag_top = w_drive + chain_len + MAG_WALL
carriage_up = mag_top  # carriage top above the dock origin
carriage_down = 6 - w_fork + CHEEK_DOWN  # under the cradle base
# Loads on the chain at full reach: it sticks out beyond the cradle's front
# edge and is held down by the sprocket
chain_free = cr_v0 - (v_deep + PUSHER_T)
chain_back = v_drive - cr_v0

# Imaging module's bed: built like a shelf on the drawer's plate, roller tops
# `bed` above the rear support's underside, so the fork works as in a lane.
# The carriage docks there like at a lane (origin at the bed's rear edge,
# centred, at the roller tops), with the drawer closed.
z_scan = im_z0 + IM_PLATE_T + MODULE_T + bed  # roller tops
z_travel1 = max(z_scan, z_deliver) + lift  # highest dock origin (fork high)
z_rail = z_travel1 + carriage_up + RAIL_CLEAR  # upper X rail (bottom)
h_back = z_rail + 18 + ROOF_T
z_pass = z_deliver + lifted_top(tine_top_high) + CLEAR  # over a fruit carried in

# Imaging module, in the carriage's (u, v, w) system at the scan dock. The
# rollers run from the front stop (v_stop) to the bed's rear edge (v = 0); a
# fruit is put down 2 mm short of the stop, like at the front of a lane. The
# side camera looks at it in +u, at its centre height, from the -u side.
scan_avo = (v_stop + 2 + along_nom / 2, rest_nom)  # nominal fruit centre (v, w)
u_bg = cr_w / 2  # background panel's inner face
cam_lens = CAM_BODY[2] / 2 + 6  # camera body centre -> lens tip
lens_gaps = [d - cam_lens - across_max / 2 for d in CAM_DIST_RANGE]
u_slide0 = -(CAM_DIST_RANGE[1] + CAM_BODY[2] / 2 + SLIDE_END)  # slide's far end
u_drive = u_bg + 5  # roller stepper, beside the bed behind the background panel
# module footprint
v_mod0 = v_stop - STOP_T - DRIVE_L - 2  # front
u_mod0 = u_slide0 - 2  # camera side
u_mod1 = u_drive + 30  # far side
scan_x = W - WALL - max(reach, u_mod1 + CLEAR)  # fruit centre (X)
# ---------------------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------------------
parts = []


def add(shape, label, color):
    """color: "#rrggbb", an (r, g, b) tuple, or ("#rrggbb", alpha)."""
    shape.label = label
    shape.color = Color(*color) if isinstance(color[1], (int, float)) else Color(color)
    parts.append(shape)
    return shape


def box(x, y, z, dx, dy, dz):
    """Box from corner (x,y,z) with size (dx,dy,dz)."""
    return Pos(x, y, z) * Box(dx, dy, dz, align=(Align.MIN, Align.MIN, Align.MIN))


def avocado_shape():
    """Nominal avocado centred at the origin, long axis along Y."""
    return scale(Sphere(1), by=(AVO_D / 2, AVO_L / 2, AVO_D / 2))


RIPE_COLORS = ["#2f3a1c", "#3f5a22", "#557a2b", "#6b9a33", "#86b83e", "#9fcf4c"]

# ---------------------------------------------------------------
# HOUSING
# ---------------------------------------------------------------
if SHOW_HOUSING:
    add(box(0, 0, 0, WALL, D, H), "Side wall L", "#d9d9d9")
    add(box(W - WALL, 0, 0, WALL, D, H), "Side wall R", "#d9d9d9")
    add(box(WALL, 0, 0, inner_w, WALL, BASE_H), "Front plinth", "#bdbdbd")
    # raised back over the elevator shaft (the chain magazine needs the
    # height); its front panel clears a fruit carried in over a top tray
    back_y0 = shelf_y1 - WALL
    pass_z = max(H, z_pass)
    back_d = D - back_y0
    add(box(WALL, D - WALL, 0, inner_w, WALL, h_back - ROOF_T), "Rear wall", "#cfcfcf")
    add(box(0, back_y0, H, WALL, back_d, h_back - H), "Raised back – side L", "#d9d9d9")
    add(
        box(W - WALL, back_y0, H, WALL, back_d, h_back - H),
        "Raised back – side R",
        "#d9d9d9",
    )
    add(
        box(WALL, back_y0, pass_z, inner_w, WALL, h_back - ROOF_T - pass_z),
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

# ---------------------------------------------------------------
# STORAGE SHELVES (no moving parts)
# ---------------------------------------------------------------
# Each level is built in a local coordinate system:
#   lx = 0..inner_w (width), ly = 0 front .. shelf_len rear (elevator),
#   lz = 0 at the rod tops.
#
# A level is a front block, a row of rods and a rear crossbar. Stored fruit
# sit in lanes from the front block backwards, newest nearest the elevator.
# The software keeps every lane's place and contents (place and size of each
# fruit, from the scan), so the fork only ever handles the rearmost:
#   Fetch:  fork low, centred on the lane -> tines slide in under the rearmost
#           fruit, in the gaps outside its two rods (the keel in the gap
#           between them) -> the elevator lifts `lift` mm -> fork retracts.
#           Nothing else moves.
#   Insert: fork high -> carries the fruit to PLACE_GAP behind the rearmost
#           one -> the elevator lowers, the fruit settles on its two rods and
#           the tines drop into their gaps -> fork retracts under it.
# A new lane starts on the first gap centre where the fruit clears its
# neighbours; it is freed when it is empty.
# Loading drawer (top level, under the customer section): staff pull it out
# and lay fruit in each lane, pushed back against the lane's fixed rear lip;
# the fork fetches from it the same way, lifting `lift_load` to clear the lip.
# The next fruit's place follows from the fetched fruit's length, measured at
# the scan. The drawer rides on fixed runners on the side walls; everything
# else on the level moves with it.


def to_world(base, lx, ly, lz):
    """Local shelf point -> global coordinates."""
    return (WALL + lx, shelf_y0 + ly, base + lz)


def rod_bed(y0=0.0):
    """Rods from ly = y0 to the rear edge, as one part."""
    length = shelf_len - y0
    rods = [
        Pos(x, y0 + length / 2, -ROD_D / 2)
        * Rot(90, 0, 0)
        * Cylinder(ROD_D / 2, length)
        for x in rod_xs
    ]
    return [(Compound(rods), "Shelf rods (stainless)", "#b0bec5")]


def bed_ends(y0=0.0):
    """Front block (holds the rods' front ends, lane stop) from ly = y0, and
    the rear crossbar the rods rest on."""
    return [
        (
            box(0, y0, -bed, inner_w, STOP_T - y0, STOP_H + bed),
            "Shelf front block (rod holder + stop)",
            "#c0392b",
        ),
        (
            box(0, shelf_len - XBAR_D, -bed, inner_w, XBAR_D, XBAR_H),
            "Shelf rear crossbar",
            "#78909c",
        ),
    ]


def drawer_front(base):
    """Insulated drawer front with handle, spanning the housing opening."""
    y = -FRONT_CLEAR - WALL
    z0 = -bed - RUNNER_H
    z1 = storage_z1 - base - 3  # up to the customer section
    zm = (z0 + z1) / 2
    return [
        (box(0, y, z0, inner_w, WALL, z1 - z0), "Loading drawer – front", "#444444"),
        (
            box(inner_w / 2 - 60, y - 18, zm - 6, 120, 18, 12),
            "Loading drawer – handle",
            "#212121",
        ),
    ]


def load_lip(cxl):
    """The loading lane's fixed lip: two posts either side of the fork notch."""
    post_w = load_half - notch
    return [
        (
            box(
                cxl + notch if sgn > 0 else cxl - load_half,
                stop_face,
                0,
                post_w,
                lip_d,
                lip_top,
            ),
            "Loading lane lip (fixed)",
            "#c0392b",
        )
        for sgn in (-1, 1)
    ]


docked = ELEV_MODE in ("fetch", "insert")
dock_origin = None
dock_is_load = False
push_face = push_rest  # pusher face in the snapshot (v)
n_fill = min(round(per_lane_nom * FILL), per_lane_nom - 1)  # leave room to insert

for s in range(n_shelves):
    is_drawer = s >= n_storage  # the top level, when LOAD_DRAWER
    # rod tops; the imaging drawer sits between the storage levels and the drawer
    base = storage_z0 + bed + s * level_pitch + (im_h if is_drawer else 0)
    T = Pos(WALL, shelf_y0, base)
    local = []

    if is_drawer:
        # drawer frame: side rails on the runners, front block up to the front
        local += rod_bed() + bed_ends(-FRONT_CLEAR) + drawer_front(base)
        for rx in (0, inner_w - RUNNER_W):
            local.append(
                (
                    box(rx, -FRONT_CLEAR, -bed, RUNNER_W, shelf_len + FRONT_CLEAR, bed),
                    "Loading drawer – side rail",
                    "#a1887f",
                )
            )
        # lane dividers, standing in the rod gaps between the lanes
        for a, b in itertools.pairwise(load_lanes):
            local.append(
                (
                    box(
                        (a + b) / 2 - DIVIDER_T / 2,
                        STOP_T,
                        -ROD_D,
                        DIVIDER_T,
                        shelf_len - STOP_T,
                        ROD_D + DIVIDER_H,
                    ),
                    "Loading drawer – lane divider",
                    "#eeeeee",
                )
            )
        lanes = load_lanes
        n = min(round(per_load_lane(along_nom) * FILL), per_load_lane(along_nom))
    else:
        local += rod_bed() + bed_ends()
        lanes = lanes_nom
        n = n_fill

    # Lanes: loading lips + avocados
    for r, cxl in enumerate(lanes):
        # the fork only works in the drawer when it is closed
        is_dock = (
            docked
            and s == DOCK_SHELF - 1
            and r == DOCK_LANE - 1
            and not (is_drawer and drawer_pull)
        )
        if is_drawer:
            local += load_lip(cxl)
            col = "#7d8a5c"  # not yet scanned
        else:
            col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        cys = queue(is_drawer, n)
        if is_dock:
            dock_origin = to_world(base, cxl, shelf_len, 0)
            dock_is_load = is_drawer
            if ELEV_MODE == "fetch" and cys:  # the rearmost is on the fork
                rear = q_rear_load if is_drawer else cys[-1] + along_nom / 2
                push_face = rear + 2 - shelf_len
                cys = cys[:-1]
            elif ELEV_MODE == "insert" and not is_drawer:  # on its way to its place
                place = STOP_T + n * (along_nom + PLACE_GAP) + along_nom - shelf_len
                push_face = push_rest + FORK_SNAPSHOT * (place - push_rest)
        lab = "Avocado (loaded, not scanned)" if is_drawer else "Avocado"
        for cy in cys:
            local.append((Pos(cxl, cy, rest_nom) * avocado_shape(), lab, col))

    if is_drawer:
        local = [(Pos(0, -drawer_pull, 0) * shp, lab, col) for shp, lab, col in local]
        # fixed parts: runners under the side rails, and the closed sensor
        for rx in (0, inner_w - RUNNER_W):
            local.append(
                (
                    box(
                        rx,
                        -FRONT_CLEAR,
                        -bed - RUNNER_H,
                        RUNNER_W,
                        shelf_len + FRONT_CLEAR,
                        RUNNER_H,
                    ),
                    "Loading drawer – runner (fixed)",
                    "#9e9e9e",
                )
            )
        local.append(
            (
                box(RUNNER_W, -FRONT_CLEAR, -bed - RUNNER_H, 12, 8, RUNNER_H),
                "Sensor: loading drawer closed",
                "#212121",
            )
        )

    for shp, lab, col in local:
        add(T * shp, lab, col)

# ---------------------------------------------------------------
# CUSTOMER SECTION ON TOP
# ---------------------------------------------------------------
sec_w = inner_w / len(TOP_SECTIONS)
top_colors = ["#2f3a1c", "#557a2b", "#86b83e"]

for i, name in enumerate(TOP_SECTIONS):
    sx = WALL + i * sec_w
    # slot at the rear end, where the fork lowers the fruit onto the tray
    tray = box(0, 0, -TRAY_T, sec_w - 6, top_len, TRAY_T) - box(
        (sec_w - 6) / 2 - notch, top_len - tine_len - 20, -13, 2 * notch, top_len, 14
    )
    tray = Pos(sx + 3, WALL, top_z) * Rot(TOP_SLOPE_DEG, 0, 0) * tray
    add(tray, f"Top section: {name}", "#8d6e63")
    # low front edge + divider
    add(
        box(sx + 3, WALL - 10, top_z - TRAY_T, sec_w - 6, 10, 45),
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
            add(Pos(ax, ay, az) * avocado_shape(), "Avocado", top_colors[i])


# ---------------------------------------------------------------
# ELEVATOR (X-Z gantry at the rear) WITH FORK CARRIAGE
# ---------------------------------------------------------------
# The carriage is built in its own system (u, v, w) with origin at the shelf's
# rear edge (lane centre, rod tops) when the carriage is docked with the fork
# low:
#   u = sideways (X), v = backwards into the shaft (Y), w = up (Z)
#
# The fork is the only actuator on the carriage: two tines with a pusher face,
# pushed and pulled by a rigid chain (rigid_chain.py). The chain runs back
# with its keel on the cradle base to a sprocket pair on its pin ends, turns a
# quarter up round it and stands in a vertical magazine, so a shallow carriage
# reaches the front of a lane. Lifting is done by the Z axis, whose two
# columns stand either side of the magazine.
# Fetch / insert: see the shelf section. The snapshots show the fork high,
#   with the rearmost avocado lifted, or on its way to its place. The chain
#   droops a little and always the same way, so the Z axis follows a droop
#   table (per fork extension) to keep the tines at their height in the lane.
# Deliver: the fork, high, reaches over the rear end of a top tray, the
#   elevator lowers so the tines pass through the tray's slot, and the fruit
#   rolls down to the customer.
# Scan (ELEV_MODE = "scan"): the carriage docks at the imaging module, in
#   its drawer under the loading drawer, puts the fruit down on its rollers as in an insert and retracts the
#   fork; it is then free for other moves while the fruit is imaged, and
#   fetches it again as from a lane. Besides ripeness, the scan gives each
#   fruit's size, which sets where it goes in a lane and where the fork picks
#   it up again. Any stored fruit can be taken back to the module to check it
#   is the expected one, or to follow its ripening.

lifted = docked and dock_origin is not None  # fork high at a lane
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


def cbox(u, v, w, du, dv, dw):
    return C * box(u, v, w, du, dv, dw)


def chain_pins(v_first, n):
    """Pin centres (v, w) of the chain in the carriage frame: from the first pin,
    straight back to the sprocket, a quarter turn up round it, then straight
    up the magazine. Each pin is one pitch (chord) from the last."""
    r, w_line = rc.r_sprocket, w_chain + rc.w_pin
    quarter = math.pi / 2 * r

    def at(t):  # t: path length from the sprocket's tangent point
        if t <= 0:
            return (v_drive + t, w_line)
        if t <= quarter:
            a = t / r
            return (v_drive + r * math.sin(a), w_drive - r * math.cos(a))
        return (v_drive + r, w_drive + t - quarter)

    t, pts = v_first - v_drive, [at(v_first - v_drive)]
    while len(pts) < n:
        lo, hi = t, t + 2 * rc.PITCH
        for _ in range(40):
            mid = (lo + hi) / 2
            q = at(mid)
            near = math.dist(q, pts[-1]) < rc.PITCH
            lo, hi = (mid, hi) if near else (lo, mid)
        t = (lo + hi) / 2
        pts.append(at(t))
    return pts


# Gantry: X rails at the bottom and top; two Z columns with linear rails,
# one each side of the chain magazine
for zr in (z_travel0 - 30, z_rail):
    add(box(WALL, D - WALL - COLUMN_T, zr, inner_w, COLUMN_T, 18), "X rail", "#607d8b")
col_z0, col_z1 = min(z_travel0 - 12, z_travel0 - carriage_down), z_rail
for sgn in (-1, 1):
    cu = ox + sgn * col_u
    add(
        box(cu - COLUMN_T / 2, oy + v_col, col_z0, COLUMN_T, COLUMN_T, col_z1 - col_z0),
        "Z column",
        "#455a64",
    )
    add(
        box(cu - 6, oy + v_zrail, col_z0, 12, Z_RAIL_T, col_z1 - col_z0),
        "Z linear rail",
        "#b0bec5",
    )

# Carriage: side cheeks from the cradle back to the Z bearing blocks, either
# side of the sprocket and magazine
cheek_u = mag_w / 2 + MAG_CLEAR
for sgn in (-1, 1):
    add(
        cbox(
            cheek_u if sgn > 0 else -cheek_u - CHEEK_T,
            cr_v1,
            -carriage_down,
            CHEEK_T,
            v_bear - cr_v1,
            carriage_up + carriage_down,
        ),
        "Carriage – side cheek",
        "#37474f",
    )
    for w0 in (-carriage_down + 5, carriage_up - 35):
        add(
            cbox(sgn * col_u - BEARING_W / 2, v_bear, w0, BEARING_W, BEARING_T, 30),
            "Bearing block",
            "#263238",
        )

# Cradle: base the tines and the chain's keel slide on, and side walls. There
# is no cradle floor - the fruit rides on the tines, over the base. The base's
# front edge is the chain's front support.
tt = tine_top_low  # tine top
add(
    cbox(-cr_w / 2, cr_v0, w_fork - 6, cr_w, cr_v1 - cr_v0, 5),
    "Cradle – base",
    "#ffb300",
)
for sgn in (-1, 1):
    u0 = cr_w / 2 if sgn > 0 else -cr_w / 2 - 4
    add(
        cbox(u0, cr_v0, w_fork - 6, 4, cr_v1 - cr_v0, tt + CR_WALL_H - w_fork + 6),
        "Cradle – side wall",
        "#ffa000",
    )
add(
    cbox(-cr_w / 2 - 12, push_rest - 20, tt + 12, 8, 14, 10),
    "Sensor: avocado in cradle",
    "#212121",
)

# Fork: tines + pusher face + chain adapter, pushed by the rigid chain.
# Below the rod tops there are only the tines (in the gaps outside the fruit's
# rods) and the keel (in the gap between them); the pusher face and the chain
# adapter sit above the rods like the chain body.
fork_w = TINE_GAP + TINE_W
head_l = HEAD_L - rc.PIN_E - rc.FIT  # chain adapter length
for sgn in (-1, 1):
    add(
        cbox(
            sgn * TINE_GAP / 2 - TINE_W / 2,
            push_face - tine_len,
            w_fork,
            TINE_W,
            tine_len + PUSHER_T,
            TINE_H,
        ),
        "Fork – tine (steel)",
        "#9e9e9e",
    )
    add(  # tine root, up into the pusher face
        cbox(
            sgn * TINE_GAP / 2 - TINE_W / 2,
            push_face,
            w_fork + TINE_H,
            TINE_W,
            PUSHER_T,
            w_chain - w_fork - TINE_H,
        ),
        "Fork – tine root",
        "#f4511e",
    )
add(
    cbox(
        -fork_w / 2,
        push_face,
        w_chain,
        fork_w,
        PUSHER_T,
        tine_top_low + PUSHER_H - w_chain,
    ),
    "Fork – pusher face",
    "#f4511e",
)
add(
    cbox(-rc.KEEL_W / 2, push_face, w_fork, rc.KEEL_W, PUSHER_T + head_l, rc.KEEL_H),
    "Fork – keel (under pusher and adapter)",
    "#f4511e",
)
v_chain0 = push_face + PUSHER_T + HEAD_L  # first pin
add(
    cbox(
        -rc.CHAIN_W / 2, push_face + PUSHER_T, w_chain, rc.CHAIN_W, head_l, rc.CHAIN_H
    ),
    "Fork – chain adapter",
    "#f4511e",
)

# Rigid chain: as many links as the stroke needs, from the fork head along
# the cradle, round the sprocket and up the magazine
n_links = math.ceil((v_drive - (push_rest + PUSHER_T + HEAD_L) + chain_len) / rc.PITCH)
pins = chain_pins(v_chain0, n_links + 1)
link, pin = rc.link_solid(), rc.pin_solid()
to_chain = Rot(0, 0, 90)  # link frame (X along, Z up) -> fork frame (v, w)
for a, b in itertools.pairwise(pins):
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    add(
        C * Pos(0, *a) * Rot(ang, 0, 0) * Pos(0, 0, -rc.w_pin) * to_chain * link,
        "Rigid chain – link",
        "#cfd8dc",
    )
for q in pins:
    add(C * Pos(0, *q) * to_chain * pin, "Rigid chain – pin", "#90a4ae")

# Drive: a sprocket pair on the pin ends (it also holds the chain down
# against the fruit's lever), motor outside a cheek, and the magazine
for sgn in (-1, 1):
    u_c = sgn * (rc.CHAIN_W / 2 + rc.FIT_U + rc.SPROCKET_T / 2)
    add(
        C
        * Pos(u_c, v_drive, w_drive)
        * Rot(0, 90, 0)
        * Cylinder(rc.r_plate, rc.SPROCKET_T),
        "Chain drive – sprocket (on the pin ends)",
        "#546e7a",
    )
add(
    cbox(cheek_u + CHEEK_T, v_drive - 15, w_drive - 15, 20, 30, 30),
    "Chain drive – motor + gears",
    "#263238",
)
add(
    cbox(-mag_w / 2, mag_v0, w_drive, mag_w, mag_v1 - mag_v0, mag_top - w_drive),
    "Chain magazine (vertical channel)",
    ("#78909c", 0.3),
)
add(
    cbox(
        -mag_w / 2,
        v_drive,
        w_fork - MAG_WALL,
        mag_w,
        mag_v1 - v_drive,
        w_drive - w_fork + MAG_WALL,
    ),
    "Chain magazine – guide round the sprocket",
    ("#78909c", 0.3),
)

# Avocado on the fork: resting in the cradle, lifted out of a lane (rear just
# ahead of the pusher face), or on its way to its place in a lane. In "scan"
# it is on the imaging module instead.
face = push_face - 2 if ELEV_MODE == "fetch" and lifted else push_face
if ELEV_MODE != "scan":
    add(
        C * Pos(0, *fruit_on_tines(*SIZES[1], face)) * avocado_shape(),
        "Avocado",
        "#6b9a33",
    )

# ---------------------------------------------------------------
# IMAGING MODULE (on the imaging level)
# ---------------------------------------------------------------
# Built in the carriage's (u, v, w) system at the scan dock: origin at the
# bed's rear edge, centred, at the roller tops. Everything stands on one base
# plate with one connector (power + data), so the module lifts out of its
# drawer as a unit. A cable loop behind the drawer lets it pull out while
# connected.
# Put down: the fork carries the fruit in high, as for an insert, to 2 mm
#   short of the front stop, lowers it onto the rollers and retracts. The
#   tines pass outside the rollers, the keel between them, and both over the
#   rear support, as in a lane.
# Spin: the rollers turn the fruit about its long axis in SPIN_STOPS steps.
#   At each stop the side camera images every LED band. Its distance is set
#   by its slide, per fruit
#   or per run, and saved with the images.
# Pick up: as a fetch.
# Open questions for the bench: grip of the rollers on the skin (bare steel
# or a thin sleeve - mind the 2 mm tine clearance), how far a lopsided fruit
# walks along the rollers, whether it stays on them without a guard on the
# camera side, and stray light from the customer section down the shaft.
# Imaging drawer: a plate on fixed runners on the side walls, with a front
# like the loading drawer's; the module moves with it
im_y0 = shelf_y0 - FRONT_CLEAR  # plate front
if n_shelves > n_storage:  # up to the loading drawer's front
    im_front_top = im_z0 + im_h - RUNNER_H - 3
else:  # up to the customer section
    im_front_top = storage_z1 - 3
im_front_z0 = im_z0 - RUNNER_H
for shp, lab, col in (
    (
        box(WALL, im_y0, im_z0, inner_w, shelf_y1 - im_y0, IM_PLATE_T),
        "Imaging drawer – plate",
        "#bdbdbd",
    ),
    (
        box(WALL, 0, im_front_z0, inner_w, WALL, im_front_top - im_front_z0),
        "Imaging drawer – front",
        "#444444",
    ),
    (
        box(
            W / 2 - 60,
            -18,
            (im_front_z0 + im_front_top) / 2 - 6,
            120,
            18,
            12,
        ),
        "Imaging drawer – handle",
        "#212121",
    ),
):
    add(Pos(0, -im_pull, 0) * shp, lab, col)
for rx in (WALL, W - WALL - RUNNER_W):
    add(
        box(rx, im_y0, im_z0 - RUNNER_H, RUNNER_W, shelf_y1 - im_y0, RUNNER_H),
        "Imaging drawer – runner (fixed)",
        "#9e9e9e",
    )
add(
    box(WALL + RUNNER_W, im_y0, im_z0 - RUNNER_H, 12, 8, RUNNER_H),
    "Sensor: imaging drawer closed",
    "#212121",
)

M = Pos(scan_x, shelf_y1 - im_pull, z_scan)


def mbox(u, v, w, du, dv, dw):
    return M * box(u, v, w, du, dv, dw)


w_base = -bed  # top of the base plate (the rear support stands on it)
add(
    mbox(u_mod0, v_mod0, w_base - MODULE_T, u_mod1 - u_mod0, -v_mod0, MODULE_T),
    "Imaging module – base plate",
    "#546e7a",
)
add(
    mbox(-cr_w / 2, -XBAR_D, w_base, cr_w, XBAR_D, XBAR_H),
    "Imaging bed – rear support (rollers rest on small bearings)",
    "#78909c",
)
for sgn in (-1, 1):
    add(
        M
        * Pos(sgn * ROD_PITCH / 2, v_stop / 2, -ROD_D / 2)
        * Rot(90, 0, 0)
        * Cylinder(ROD_D / 2, -v_stop),
        "Imaging bed – roller (driven)",
        "#90a4ae",
    )
add(
    mbox(u_bg, v_stop, w_base, 3, -v_stop, fruit_top_max + BG_UP - w_base),
    "Imaging bed – background panel (matte, also the far-side guard)",
    "#37474f",
)
add(  # on the panel, above the largest fruit
    mbox(u_bg - 1, scan_avo[0] - 20, fruit_top_max + 3, 1, 40, BG_UP - 6),
    "Reference tile (white / grey)",
    "#f5f5f5",
)
add(
    mbox(-STOP_W / 2, v_stop - STOP_T, w_base, STOP_W, STOP_T, bed + BED_STOP_H),
    "Imaging bed – front stop (roller bearings)",
    "#c0392b",
)
v_belt = v_stop - STOP_T - DRIVE_L
add(
    mbox(-STOP_W / 2, v_belt, w_base, u_drive + 28 + STOP_W / 2, DRIVE_L, bed + 8),
    "Imaging bed – roller belt (cover)",
    "#455a64",
)
add(
    mbox(u_drive, v_belt + DRIVE_L, w_base, 28, 45, 30),
    "Imaging bed – roller stepper",
    "#263238",
)
add(
    mbox(u_mod0, v_mod0, w_base, 25, 15, 12),
    "Imaging module – connector (power + data)",
    "#212121",
)

# Cameras: Pi camera + LED holder, built looking along -z
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
side_cam = M * Pos(-CAM_DIST, scan_avo[0], scan_avo[1]) * Rot(0, -90, 0)  # looks +u
for shp, lab, col in cam:
    add(side_cam * shp, "Side camera – " + lab, col)

# Distance slide for the side camera: a rail with a lead screw along u on the
# base plate, a stepper at its far end, and a carriage under the camera
u_near = -(CAM_DIST_RANGE[0] - cd / 2)  # camera front at its nearest
add(
    mbox(
        u_slide0 + SLIDE_END,
        scan_avo[0] - 10,
        w_base,
        u_near - u_slide0 - SLIDE_END,
        20,
        8,
    ),
    "Camera slide – rail + lead screw",
    "#78909c",
)
add(
    mbox(u_slide0, scan_avo[0] - 14, w_base, SLIDE_END, 28, 30),
    "Camera slide – stepper",
    "#263238",
)
add(
    mbox(-CAM_DIST - 12, scan_avo[0] - 12, w_base + 8, 24, 24, -w_base - 8),
    "Camera slide – carriage",
    "#455a64",
)

if ELEV_MODE == "scan":
    add(M * Pos(0, *scan_avo) * avocado_shape(), "Avocado (being imaged)", "#6b9a33")

# ---------------------------------------------------------------
# EXPORT + REPORT
# ---------------------------------------------------------------
model = Compound(children=parts, label="Avocado Manager")

if __name__ == "__main__":
    export_step(model, "avocado_manager.step")

    print("=" * 56)
    print(" FRUIT AND ROD BED")
    print("=" * 56)
    print(
        f" Fruit size range:          D {AVO_D_RANGE[0]}–{AVO_D_RANGE[1]},"
        f" L {AVO_L_RANGE[0]}–{AVO_L_RANGE[1]} mm  (nominal {AVO_D} x {AVO_L})"
    )
    print(
        f" Rods:                      {n_rods} x Ø{ROD_D} at {ROD_PITCH} mm pitch"
        f" ({rod_gap} mm gaps), {shelf_len:.0f} mm long"
    )
    print(f" Shelf thickness:           {bed} mm  (rods + rear crossbar {XBAR_H} mm)")
    print(f" Fruit bottom vs rod tops:  {floor_z:+.1f} mm  (sinks into its gap)")
    print(
        f" Drawer lanes:              {load_pitch_x} mm pitch, {2 * load_half:.0f} mm"
        f" inside, dividers {DIVIDER_H} mm high"
    )
    print("=" * 56)
    print(" FORK IN THE ROD BED (fork low)")
    print("=" * 56)
    print(
        f" Tines:                     {TINE_W} x {TINE_H} mm steel, {TINE_GAP} mm"
        f" apart, top {tine_top_low:+.1f} mm vs rod tops"
    )
    print(
        f" Side clearance to rods:    tines {tine_side:.1f} mm, keel {keel_side:.1f} mm"
        "  (X positioning)"
    )
    if min(tine_side, keel_side) < 1.5:
        print("   ! less than 1.5 mm: the fork may catch the rods")
    print(f" Underside over crossbar:   {xbar_gap:.1f} mm  (tines and keel)")
    if xbar_gap < XBAR_CLEAR:
        print(
            "   ! the tines / keel hit the rear crossbar: thicker rods or lower tines"
        )
    print(f" Chain body over rod tops:  {chain_gap:.1f} mm  (keel {rc.KEEL_H} mm)")
    if chain_gap < CHAIN_CLEAR:
        print("   ! the chain body drags on the rods: deeper keel")
    print(f" Fruit over cradle base:    {base_gap:.1f} mm  (chain's front support)")
    if base_gap < 1:
        print("   ! a fruit on the tines hits the cradle's front edge")
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
        f" Lift per fetch / insert:   {lift:.0f} mm"
        + (f", loading drawer {lift_load:.0f} mm" if LOAD_DRAWER else "")
        + "  (elevator Z)"
    )
    print(f" Fork reach (chain):        {fork_reach:.0f} mm  (rest -> front of a lane)")
    print(f" Cradle pocket (W x D):     {cr_w:.0f} x {push_rest - cr_v0:.0f} mm")
    print(f" Elevator shaft depth:      {elev_depth:.0f} mm")
    if LOAD_DRAWER:
        lip_rel = [lip_top - (rest_height(ac, h) - h / 2) for ac, _, h in SIZES]
        print(
            f" Loading lip:               {min(lip_rel):.0f}–{max(lip_rel):.0f} mm"
            f" above fruit bottom, climbing takes {hold_min:.2f} x fruit weight"
        )
    print("=" * 56)
    print(" RIGID CHAIN (details and printable parts: rigid_chain.py)")
    print("=" * 56)
    for line in rc.report_lines(
        chain_free,
        chain_back,
        fruit_lens={"Avocado (Hass)": along_max},
        head_g=HEAD_G,
        pusher_t=PUSHER_T,
    ):
        print(line)
    print(f" Links / pins:              {n_links}  (stroke {chain_len:.0f} mm)")
    print(
        f" Magazine (D x H):          {mag_v1 - mag_v0:.0f} x {mag_top - w_drive:.0f} mm,"
        f" vertical, behind the sprocket"
    )
    print(
        f" Carriage:                  {carriage_up:.0f} mm above, {carriage_down:.0f}"
        f" mm below the fork's dock height, {2 * (col_u + COLUMN_T / 2):.0f} mm wide"
    )
    print("=" * 56)
    print(" IMAGING DRAWER AND MODULE")
    print("=" * 56)
    print(
        f" Height at the back:        {h_back:.0f} mm  (front {H} mm; set by the chain"
        " magazine)"
    )
    print(
        f" Imaging drawer:            level height {im_h:.0f} mm (a storage level"
        f" {level_pitch:.0f}), "
        + ("under the loading drawer" if LOAD_DRAWER else "on top")
        + f", pulls out {shelf_len - DRAWER_KEEP:.0f} mm"
    )
    print(
        f" Imaging module:            {u_mod1 - u_mod0:.0f} mm wide, on the right"
        f" ({scan_x + u_mod0 - WALL:.0f} mm free on the left)"
    )
    if scan_x + u_mod0 < WALL:
        print("   ! the imaging module is wider than the machine")
    print(
        f" Imaging bed:               2 rollers Ø{ROD_D} at {ROD_PITCH} mm,"
        f" {-v_stop:.0f} mm long, top z = {z_scan:.0f}"
    )
    print(
        f" Spin:                      {across_min / ROD_D:.1f}–{across_max / ROD_D:.1f}"
        " roller turns per fruit turn"
    )
    print(
        f" Rotation stops:            {SPIN_STOPS}  (all of the circumference within"
        f" {180 / SPIN_STOPS:.0f}° of the side view)"
    )
    print(
        f" Side camera (slide):       {CAM_DIST_RANGE[0]}–{CAM_DIST_RANGE[1]} mm from"
        f" the fruit centre (shown at {CAM_DIST})"
    )
    print(f" Lens gap to largest fruit: {lens_gaps[0]:.0f}–{lens_gaps[1]:.0f} mm")
    if lens_gaps[0] < 10:
        print("   ! the side camera can come too close to a large fruit")
    if not CAM_DIST_RANGE[0] <= CAM_DIST <= CAM_DIST_RANGE[1]:
        print("   ! CAM_DIST is outside the slide range")
    print("=" * 56)
    print(" CAPACITY REPORT")
    print("=" * 56)
    print(f" Shelf area (W x D):        {inner_w:.0f} x {shelf_len:.0f} mm")
    print(
        f" Lanes per shelf:           {len(lanes_nom)}"
        f"  (smallest fruit {len(lanes_min)}, largest {len(lanes_max)};"
        f" elevator reaches {reach:.0f}–{inner_w - reach:.0f})"
    )
    print(
        f" Avocados per lane:         {per_lane_nom}"
        f"  (smallest fruit {per_lane(along_min)}, largest {per_lane(along_max)})"
    )
    print(f" Avocados per shelf:        {len(lanes_nom) * per_lane_nom}")
    print(
        f" Level height:              {level_pitch:.0f} mm"
        + (f"  (loading drawer {load_pitch:.0f})" if load_pitch > level_pitch else "")
    )
    print(f" Levels that fit:           {n_levels_fit}  (desired {N_SHELVES})")
    print(
        f" Storage shelves in model:  {n_storage}"
        + ("  + loading drawer on top" if LOAD_DRAWER and n_shelves else "")
    )
    print(f" Total storage capacity:    {n_storage * len(lanes_nom) * per_lane_nom}")
    print(
        f"   (all smallest / largest: {n_storage * len(lanes_min) * per_lane(along_min)}"
        f" / {n_storage * len(lanes_max) * per_lane(along_max)})"
    )
    if LOAD_DRAWER:
        n_load = len(load_lanes)
        print(
            f" Loading drawer buffer:     {n_load * per_load_lane(along_nom)}"
            f"  (largest fruit {n_load * per_load_lane(along_max)}),"
            f" pulls out {shelf_len - DRAWER_KEEP:.0f} mm"
        )
    if n_shelves < N_SHELVES:
        need = H + load_pitch + (N_SHELVES - 1) * level_pitch - avail_h
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
