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
    Plane,
    Polygon,
    Pos,
    Rot,
    Sphere,
    export_step,
    extrude,
    scale,
)

# ---------------------------------------------------------------
# PARAMETERS
# ---------------------------------------------------------------
# Outer dimensions
W, D, H = 600, 700, 800
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
ROD_D = 8  # rod diameter (stainless, standard stock)
ROD_PITCH = 18  # centre distance between rods
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
# fruit are pushed back against a gate - a bar across the drawer's rear end at
# the fruits' centre height, so a push can't tip a fruit over it. As the drawer
# closes, fixed ramps on the side walls lift the gate clear of the fork, which
# then fetches as from a storage lane; as it opens, the gate drops back by its
# own weight. The drawer front closing against the housing positions the lanes
# for the fork; the elevator only fetches from the drawer when a sensor reports
# it closed.
LOAD_DRAWER = True  # False: no loading level (all levels are storage)
DIVIDER_T = 4  # lane divider (stands in a rod gap, or on a rod)
DIVIDER_H = 35  # divider height above the rod tops
GATE_T = 6  # rear gate: bar thickness along the lanes
GATE_LAP = 8  # ... reaches this far below and above the fruit centres it holds
GUIDE_W = 6  # gate guides, on the inside of the drawer's side rails
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
TINE_W = 5  # tine width (5 x 5 key steel)
TINE_H = 5  # tine height: deep enough that a small fruit, sagging between the
# tines, clears the cradle base, and shallow enough to clear the crossbar
TINE_CLEAR = 3  # clearance tine <-> fruit when sliding under it
XBAR_CLEAR = 1  # min gap between the fork's underside and the rear crossbar
CHAIN_CLEAR = 1.5  # min gap between the chain body and the rod tops, fork low
LIFT_CLEAR = 3  # a carried fruit clears the rods (or rollers) by this
TIP_MARGIN = 8  # the tip stays this far ahead of a fruit's centre and behind the next
PUSHER_T = 6  # pusher face thickness
PUSHER_H = 30  # pusher face height above the tines
HEAD_L = 10  # chain adapter behind the pusher face, up to the first pin
HEAD_G = 40  # fork head (pusher + tines) mass, g

# Elevator (rear side). The shaft depth follows from the carriage stack below.
ELEV_MODE = "insert"  # "fetch":  the fork has lifted the rearmost avocado of a lane
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
# cheeks; the shaft depth follows from the sprocket and magazine. Its widest
# part sets how close to a side wall a lane can be.
CR_GAP = 5  # gap between the shelves' rear edge and the cradle
CR_WALL_H = 50  # cradle side wall height above the tines
CR_WALL_T = 4  # cradle side wall thickness
SIDE_CLEAR = 3  # carriage <-> side walls, at the outermost lanes
DRIVE_MOTOR = (20, 30, 30)  # chain drive motor + gears beside a cheek: width,
# depth, height
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
# the fork serves it like a lane (a short bed of two rollers). Staff pull the
# drawer out to clean, calibrate or swap the module. It is inside the closed
# body, so it is dark and at storage temperature. It rides on runners like the
# loading drawer.
IM_PLATE_T = 6  # imaging drawer's plate
MODULE_T = 4  # module base plate, bolted onto the drawer's plate
IM_DRAWER_OPEN = 0.0  # snapshot: how far the imaging drawer is pulled out (0..1)
RAIL_CLEAR = 15  # clearance between the carriage top and the upper X rail
ROOF_T = 20  # roof of the raised back (over the elevator shaft)

# Imaging module: two driven rollers spin the fruit about its long axis in
# front of a side camera on a motorised distance slide. They are thick, with a
# silicone sleeve for grip, and set wide apart, so the fruit sits deep between
# them and the fork's tines pass inside them. There is no guard on the camera
# side; a matte background panel on the far side carries the reference tile.
# A narrow front stop keeps the spinning fruit from walking. The fruit's ends
# are only seen at a slant; an end camera can be added later.
IM_ROLLER_D = 18  # roller diameter (steel tube + silicone sleeve)
IM_ROLLER_CLEAR = 4  # side clearance tine <-> roller
BED_SPARE = 10  # bed length beyond the longest fruit
BED_STOP_H = 15  # front stop above the roller tops
STOP_W = 30  # front stop width, where it rises above the rollers
BG_UP = 20  # background panel reaches this far above the largest fruit
DRIVE_L = 15  # roller belt in front of the stop (its stepper is beside the bed)
SPIN_STOPS = 6  # rotation stops per fruit (all LED bands are imaged at each)
CAM_BODY = (60, 60, 25)  # camera + LED holder: width, height, depth (view axis)
CAM_DIST_RANGE = (110, 220)  # side camera's slide range, its centre to the fruit's
CAM_DIST = 140  # side camera distance in the snapshot (within CAM_DIST_RANGE)
SLIDE_END = 35  # slide's stepper, beyond the camera's farthest position

# Customer section on top: trays sloping towards the customer. A fruit lies
# on them lengthwise, so it can't roll down - it slides, and the slope follows
# from the friction between skin and tray.
TOP_SECTIONS = ["Soft ripe", "Firm ripe", "Ripe in a few days"]
TRAY_MU = 0.3  # friction fruit <-> tray: tan of the angle where a fruit starts
# to slide on a tilted tray (measure it; a PTFE or UHMW-PE surface lowers it)
TRAY_SLIP_DEG = 3  # tray slope beyond that angle, so every fruit gets going
TRAY_T = 12  # tray thickness
TOP_DIV_H = 40  # section dividers' height above the tray

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
def rest_height(across, height, r=ROD_D / 2, xs=None):
    """Fruit centre above the tops of the rods (or rollers) it rests on, radius
    r at ±x for each x in xs - by default the bed's rods, with the fruit
    centred over a gap between two. Its cross-section across the lane is
    treated as an ellipse (across x height). Bisection for the lowest centre
    where it clears every rod under it."""
    a, b = across / 2, height / 2
    if xs is None:
        xs = [(k + 0.5) * ROD_PITCH for k in range(int(a // ROD_PITCH) + 1)]
    ts = [2 * math.pi * i / 720 for i in range(720)]

    def clears(z_c):  # rod centres at (±x, -r); symmetric, so +x only
        return all(
            (x / a) ** 2 + ((r + z_c) / b) ** 2 > 1  # rod centre outside the fruit
            and all(
                math.hypot(a * math.cos(t) - x, z_c + b * math.sin(t) + r) >= r
                for t in ts
            )
            for x in xs
        )

    lo, hi = -r, b  # centre level with the rods' .. resting on their tops
    assert not clears(lo), "fruit falls between the rods"
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

# Carriage width: the Z columns stand either side of the chain magazine, the
# drive motor beside a cheek, and the cradle is as wide as a drawer lane. A
# lane centre must be this far from a side wall for the elevator to reach it.
lane_inner = across_max + 2 * CLEAR  # drawer lane and cradle inner width
cr_w = lane_inner  # cradle inner width
mag_w = rc.pin_len + 2 * MAG_WALL  # magazine width, pin ends included
cheek_u = mag_w / 2 + MAG_CLEAR  # side cheeks' inner faces
col_u = cheek_u + BEARING_W / 2  # column centre, sideways
car_half = max(
    col_u + max(COLUMN_T, BEARING_W) / 2,  # Z columns, bearing blocks
    cheek_u + CHEEK_T + DRIVE_MOTOR[0],  # chain drive motor
    cr_w / 2 + CR_WALL_T,  # cradle side walls
)
reach = car_half + SIDE_CLEAR

# Rod bed: rods across the inner width, with a gap centre at `reach`, so the
# outermost lanes sit as close to the side walls as the carriage allows. Lane
# centres are gap centres.
rod_gap = ROD_PITCH - ROD_D
rod_x0 = reach - ROD_PITCH / 2
rod_x0 -= ROD_PITCH * ((rod_x0 - ROD_EDGE) // ROD_PITCH)  # leftmost rod
n_rods = int((inner_w - ROD_EDGE - rod_x0) // ROD_PITCH) + 1
rod_xs = [rod_x0 + i * ROD_PITCH for i in range(n_rods)]
gap_xs = [x + ROD_PITCH / 2 for x in rod_xs[:-1]]
bed = ROD_D + XBAR_H  # rod tops -> underside of the rear crossbar

# Tines in the gaps just outside the fruit's two rods; the chain's keel in the
# gap between them. The slot in the top trays' rear ends lets the whole fork
# head through.
TINE_GAP = 2 * ROD_PITCH  # centre distance between the two tines
tine_in = TINE_GAP / 2 - TINE_W / 2  # inner edge of a tine (the fruit rests on it)
notch = TINE_GAP / 2 + TINE_W / 2 + TINE_CLEAR  # half width of the slots
tine_side = (rod_gap - TINE_W) / 2  # sideways clearance tine <-> rod
keel_side = (rod_gap - rc.KEEL_W) / 2  # ... keel <-> rod


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

# Loading drawer lanes: fixed, for the largest fruit, on gap centres (the fork
# works there as in a storage lane). The lane pitch is a whole number of rod
# pitches, so a divider between two lanes stands in a rod gap, or - when that
# number is odd - on a rod.
load_pitch_x = ROD_PITCH * math.ceil((lane_inner + DIVIDER_T) / ROD_PITCH)
div_on_rod = round(load_pitch_x / ROD_PITCH) % 2 == 1
load_half = (load_pitch_x - DIVIDER_T) / 2  # half the drawer lane's inner width
load_edge = max(reach, load_half)  # nearest drawer lane centre to a side wall
load_lanes = []
for x in gap_xs:
    next_free = not load_lanes or x - load_lanes[-1] > load_pitch_x - 0.1
    if next_free and load_edge <= x <= inner_w - load_edge:
        load_lanes.append(x)

# Fork heights above the rod tops. Low: the tines slide in under a fruit.
# High: a fruit on the tines (it sags between them) clears the rods.
tine_top_low = (
    min(rest_height(ac, h) - on_points(ac, h, tine_in) for ac, _, h in SIZES)
    - TINE_CLEAR
)
tine_top_high = max(
    rest_height(ac, h) + LIFT_CLEAR - on_points(ac, h, tine_in) for ac, _, h in SIZES
)
lift = tine_top_high - tine_top_low  # Z move of the elevator per fetch/insert


def lifted_top(tine_top):
    """Top of the largest fruit on the tines, above the rod tops."""
    return max(tine_top + on_points(ac, h, tine_in) + h / 2 for ac, _, h in SIZES)


# Loading drawer's rear gate, above the rod tops. Down, it covers every fruit
# size's centre height, so a fruit pushed against it can't climb it; the ramps
# lift it clear of a fruit on the fork (45° ramp, then a flat it rests on when
# the drawer is closed).
rests = [rest_height(ac, h) for ac, _, h in SIZES]
gate_z0 = min(rests) - GATE_LAP  # underside, down
gate_h = max(rests) + GATE_LAP - gate_z0
gate_rise = lifted_top(tine_top_high) + CLEAR - gate_z0
ramp_run = gate_rise + GATE_T  # drawer travel over which the gate drops


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
q_rear_load = shelf_len - GATE_T  # rear point of a fruit against the gate
v_deep = STOP_T + along_min + 2 - shelf_len  # pusher face at the frontmost fruit
fork_reach = push_rest - v_deep  # fork stroke, rest -> front of a lane


def per_lane(along):
    return int((shelf_len - STOP_T + PLACE_GAP) // (along + PLACE_GAP))


def per_load_lane(along):
    return int((q_rear_load - STOP_T) // along)


def queue(is_load, n):
    """Centres (ly) of n nominal fruit in a lane, front to rear. Stored fruit
    sit from the front stop with gaps; the loading lane is pushed in by hand
    against the rear gate."""
    if is_load:
        return [q_rear_load - along_nom / 2 - k * along_nom for k in range(n)][::-1]
    return [STOP_T + along_nom / 2 + k * (along_nom + PLACE_GAP) for k in range(n)]


per_lane_nom = per_lane(along_nom)

# Level height: rod tops to rod tops. A lifted fruit leaving a lane passes
# under the rear crossbar of the level above, so the whole bed counts.
level_pitch = max(fruit_top_max, lifted_top(tine_top_high)) + CLEAR + bed
storage_z0 = BASE_H
storage_z1 = H - TOP_H  # underside of the customer section at the front

# Customer section on top
top_z = H - TOP_H + 20
top_len = shelf_y1 - WALL - 5
top_slope = math.atan(TRAY_MU) + math.radians(TRAY_SLIP_DEG)
top_slope_deg = math.degrees(top_slope)
top_rise = top_len * math.sin(top_slope)  # a tray's rear end above its front
# the section dividers' top, at the trays' front and at the raised back;
# where it rises above the side walls, they get cheeks
div_top0 = top_z + TOP_DIV_H / math.cos(top_slope)
div_back = div_top0 + (shelf_y1 - 2 * WALL) * math.tan(top_slope)

# Imaging module's bed: two rollers at ±im_roll_u, the tines running inside
# them and the keel between them. The fruit sits deep between the rollers, so
# the carriage docks there `im_drop` below the roller tops (origin at the bed's
# rear edge, centred), and the fork works as in a lane. Heights here are
# above the roller tops.
im_roll_r = IM_ROLLER_D / 2
im_roll_u = TINE_GAP / 2 + TINE_W / 2 + IM_ROLLER_CLEAR + im_roll_r


def rest_im(across, height):
    """Fruit centre above the roller tops."""
    return rest_height(across, height, im_roll_r, (im_roll_u,))


rest_nom_im = rest_im(across_nom, AVO_D)
fruit_top_max_im = max(rest_im(ac, h) + h / 2 for ac, _, h in SIZES)
tine_top_low_im = (
    min(rest_im(ac, h) - on_points(ac, h, tine_in) for ac, _, h in SIZES) - TINE_CLEAR
)
tine_top_high_im = max(
    rest_im(ac, h) + LIFT_CLEAR - on_points(ac, h, tine_in) for ac, _, h in SIZES
)
lift_im = tine_top_high_im - tine_top_low_im
im_drop = tine_top_low - tine_top_low_im  # dock origin below the roller tops
# the rear support's crossbar is as far under the fork as in a lane
bed_im = xbar_gap + XBAR_H - w_fork + im_drop  # roller tops -> base plate top
# sideways push (x the fruit's weight) that rolls a fruit up over a roller
hold_im = min(im_roll_u / (rest_im(ac, h) + im_roll_r) for ac, _, h in SIZES)

# Imaging drawer, on top of the storage levels (under the loading drawer).
# The module's bed runs from its front stop (v_stop, in the carriage's v at
# the scan dock) to the shelves' rear edge.
v_stop = -(along_max + BED_SPARE)
w_mod = max(  # module's height above the roller tops
    lifted_top(tine_top_high_im),  # a fruit carried in
    fruit_top_max_im + BG_UP,  # background panel
    rest_nom_im + CAM_BODY[0] / 2,  # side camera
)
# its level: plate underside -> underside of the next level's bed
im_h = IM_PLATE_T + MODULE_T + bed_im + w_mod + CLEAR
avail_h = storage_z1 - storage_z0 - im_h  # for the rod-bed levels
n_levels_fit = max(int(avail_h // level_pitch), 0)
n_shelves = min(N_SHELVES, n_levels_fit)  # levels in the model, incl. the drawer
n_storage = n_shelves - 1 if LOAD_DRAWER and n_shelves else n_shelves
drawer_pull = min(max(DRAWER_OPEN, 0), 1) * (shelf_len - DRAWER_KEEP)
im_z0 = storage_z0 + n_storage * level_pitch  # imaging drawer plate underside
im_pull = min(max(IM_DRAWER_OPEN, 0), 1) * (shelf_len - DRAWER_KEEP)
# the loading drawer's gate, lifted, stays under the top trays
load_base = storage_z0 + bed + n_storage * level_pitch + im_h  # its rod tops
gate_top_up = load_base + gate_z0 + gate_rise + gate_h
tray_under_gate = (
    top_z
    - TRAY_T / math.cos(top_slope)
    + (shelf_y1 - GATE_T - WALL) * math.tan(top_slope)
)


# Elevator travel. The carriage is built around a dock origin at the shelf's
# rear edge (lane centre, rod tops): there the fork is low, `lift` higher it
# is high.
def fruit_on_tines(across, along, height, face):
    """(v, w) of a fruit's centre on the tines, rear against the pusher face."""
    return face - along / 2, tine_top_low + on_points(across, height, tine_in)


x_travel0 = WALL + reach
x_travel1 = W - WALL - reach
z_travel0 = storage_z0 + bed  # lowest dock origin (the bottom level's rod tops)
z_deliver = top_z + top_rise  # rear end of the top trays

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

# The carriage docks at the imaging module (drawer closed) as at a lane
z_scan = im_z0 + IM_PLATE_T + MODULE_T + bed_im  # roller tops
z_scan_dock = z_scan - im_drop  # dock origin there (fork low)
z_travel1 = max(z_scan_dock + lift_im, z_deliver + lift)  # highest dock (fork high)
z_rail = z_travel1 + carriage_up + RAIL_CLEAR  # upper X rail (bottom)
h_back = z_rail + 18 + ROOF_T
z_pass = z_deliver + lifted_top(tine_top_high) + CLEAR  # over a fruit carried in

# Imaging module, in a (u, v, w) system like the carriage's at the scan dock,
# but with w = 0 at the roller tops. The rollers run from the front stop
# (v_stop) to the bed's rear edge (v = 0); a fruit is put down 2 mm short of
# the stop, like at the front of a lane. The side camera looks at it in +u, at
# its centre height, from the -u side.
scan_avo = (v_stop + 2 + along_nom / 2, rest_nom_im)  # nominal fruit centre (v, w)
u_bg = max(cr_w / 2, im_roll_u + im_roll_r + CLEAR)  # background panel, inner face
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


def prism(x, dx, profile):
    """Prism from a side profile [(y, z), ...], from x to x + dx."""
    return Pos(x, 0, 0) * extrude(Plane.YZ * Polygon(*profile, align=None), amount=dx)


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
    # cheeks on the side walls, up to the top of the trays' dividers where
    # they rise above the walls
    if div_back > H:
        y_h = max(WALL + (H - div_top0) / math.tan(top_slope), 0)
        cheek = [(y_h, H), (back_y0, H), (back_y0, div_back)]
        if y_h == 0:  # the dividers are above the walls already at the front
            cheek.append((0, div_top0 - WALL * math.tan(top_slope)))
        for x, side in ((0, "L"), (W - WALL, "R")):
            add(prism(x, WALL, cheek), f"Top section – side cheek {side}", "#d9d9d9")
    # front door over the storage levels (drawn see-through); opening it
    # stops the elevator
    door_z0, door_z1 = BASE_H + 3, im_z0 - RUNNER_H - 3
    add(
        box(WALL, 0, door_z0, inner_w, WALL, door_z1 - door_z0),
        "Front door (storage levels)",
        ("#444444", 0.25),
    )
    add(
        box(W - WALL - 30, -18, (door_z0 + door_z1) / 2 - 40, 12, 18, 80),
        "Front door – handle",
        "#212121",
    )
    for z in (door_z0 + 30, door_z1 - 30):
        add(Pos(WALL + 4, -4, z) * Cylinder(4, 40), "Front door – hinge", "#9e9e9e")
    add(
        box(W - WALL - 20, WALL, door_z1 - 15, 12, 8, 12),
        "Sensor: front door closed (interlock: open stops the elevator)",
        "#212121",
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
# and lay fruit in each lane, pushed back against the rear gate. Closing the
# drawer lifts the gate (a follower on each end rides up a fixed ramp), and
# the fork fetches as from a storage lane. The next fruit's place follows from
# the fetched fruit's length, measured at the scan. The drawer rides on fixed
# runners on the side walls; everything else on the level but the ramps moves
# with it.


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


def load_gate(up):
    """The loading drawer's rear gate, lifted `up` mm: the bar between two
    guides on the side rails, with a follower on each end over the ramps."""
    z, z_top = gate_z0 + up, gate_z0 + gate_rise + gate_h
    out = [
        (
            box(
                RUNNER_W + GUIDE_W,
                q_rear_load,
                z,
                inner_w - 2 * (RUNNER_W + GUIDE_W),
                GATE_T,
                gate_h,
            ),
            "Loading drawer – rear gate",
            "#c0392b",
        )
    ]
    for gx, fx in ((RUNNER_W, 1), (inner_w - RUNNER_W - GUIDE_W, inner_w - RUNNER_W)):
        out += [
            (
                box(gx, q_rear_load, 0, GUIDE_W, GATE_T, z_top),
                "Loading drawer – gate guide",
                "#a1887f",
            ),
            (
                box(fx, q_rear_load, z, RUNNER_W - 1, GATE_T, GATE_T),
                "Loading drawer – gate follower",
                "#212121",
            ),
        ]
    return out


# The ramps (fixed, on the side walls above the drawer's side rails), in the
# closed drawer's frame: 45° up to a flat under the followers
ly_follow = q_rear_load + GATE_T / 2
ramp = [
    (ly_follow - ramp_run, gate_z0),
    (shelf_len, gate_z0),
    (shelf_len, gate_z0 + gate_rise),
    (ly_follow - GATE_T, gate_z0 + gate_rise),
]
gate_up = gate_rise * min(max((ramp_run - drawer_pull) / gate_rise, 0), 1)

docked = ELEV_MODE in ("fetch", "insert")
dock_origin = None
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
        # lane dividers between the lanes, up to the gate: standing in a rod
        # gap, or clipped onto a rod
        div_z0 = 0 if div_on_rod else -ROD_D
        for a, b in itertools.pairwise(load_lanes):
            local.append(
                (
                    box(
                        (a + b) / 2 - DIVIDER_T / 2,
                        STOP_T,
                        div_z0,
                        DIVIDER_T,
                        q_rear_load - STOP_T,
                        DIVIDER_H - div_z0,
                    ),
                    "Loading drawer – lane divider",
                    "#eeeeee",
                )
            )
        local += load_gate(gate_up)
        lanes = load_lanes
        n = min(round(per_load_lane(along_nom) * FILL), per_load_lane(along_nom))
    else:
        local += rod_bed() + bed_ends()
        lanes = lanes_nom
        n = n_fill

    # Lanes: avocados
    for r, cxl in enumerate(lanes):
        # the fork only works in the drawer when it is closed
        is_dock = (
            docked
            and s == DOCK_SHELF - 1
            and r == DOCK_LANE - 1
            and not (is_drawer and drawer_pull)
        )
        if is_drawer:
            col = "#7d8a5c"  # not yet scanned
        else:
            col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        cys = queue(is_drawer, n)
        if is_dock:
            dock_origin = to_world(base, cxl, shelf_len, 0)
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
        for rx in (0, inner_w - RUNNER_W):
            local.append(
                (
                    prism(rx, RUNNER_W, ramp),
                    "Loading drawer – gate ramp (fixed)",
                    "#9e9e9e",
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
    tray = Pos(sx + 3, WALL, top_z) * Rot(top_slope_deg, 0, 0) * tray
    add(tray, f"Top section: {name}", "#8d6e63")
    # low front edge + divider
    add(
        box(sx + 3, WALL - 10, top_z - TRAY_T, sec_w - 6, 10, 45),
        f"Front edge {name}",
        "#5d4037",
    )
    on_tray = Pos(sx, WALL, top_z) * Rot(top_slope_deg, 0, 0)
    add(  # its front end cut square, behind the front edge
        on_tray * box(0, 0, 0, 6, top_len, TOP_DIV_H) - box(sx - 1, 0, 0, 8, WALL, H),
        "Top divider",
        "#5d4037",
    )
    # avocados slid down against the front edge, lying on the tray: the first
    # row's centre (s along the tray) puts the fruit's front at the edge
    sin_s, cos_s = math.sin(top_slope), math.cos(top_slope)
    half_y = math.hypot(AVO_L / 2 * cos_s, AVO_D / 2 * sin_s)  # fruit, half in Y
    s_front = (half_y + AVO_D / 2 * sin_s) / cos_s
    n_side = int((sec_w - 20) // (AVO_D + 5))
    for rr in range(2):
        for c in range(n_side):
            ax = 10 + (AVO_D + 5) / 2 + c * (AVO_D + 5)
            at = Pos(ax, s_front + rr * (AVO_L + 5), AVO_D / 2)
            add(on_tray * at * avocado_shape(), "Avocado", top_colors[i])


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
#   slides down to the customer.
# Scan (ELEV_MODE = "scan"): the carriage docks at the imaging module, in its
#   drawer under the loading drawer (a little lower than at a lane: the fruit
#   sits deeper between the rollers), puts the fruit down on the rollers as in
#   an insert and retracts the fork; it is then free for other moves while the
#   fruit is imaged, and fetches it again as from a lane. Besides ripeness,
#   the scan gives each fruit's size, which sets where it goes in a lane and
#   where the fork picks it up again. Any stored fruit can be taken back to the
#   module to check it is the expected one, or to follow its ripening.

lifted = docked and dock_origin is not None  # fork high at a lane
if ELEV_MODE == "scan":
    dock_origin = (scan_x, shelf_y1, z_scan_dock)
if dock_origin is None:  # "free" or invalid dock position
    dock_origin = (
        x_travel0 + ELEV_X * (x_travel1 - x_travel0),
        shelf_y1,
        z_travel0 + ELEV_Z * (z_travel1 - z_travel0),
    )
ox, oy, oz = dock_origin
C = Pos(ox, oy, oz + (lift if lifted else 0))


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
# Sensors, so the machine measures where things are instead of assuming it,
# and finds its way again after a power cut or after staff have moved fruit:
# - a light beam across the cradle mouth, above the chain and pusher, so only
#   a fruit breaks it: a fruit is all the way in before the carriage moves,
#   and its length is checked as it passes;
# - a rod finder (reflective IR) at the base's front edge, looking at the
#   rods' rear ends: sweeping X, and Z at one rod, finds each level's rods
#   (and the imaging bed's rollers), so the elevator calibrates itself there;
# - a ToF distance sensor in the pusher face (with the fork, below).
beam_w = max(w_chain + rc.CHAIN_H, tt + PUSHER_H) + CLEAR  # beam height
for sgn in (-1, 1):
    u0 = cr_w / 2 if sgn > 0 else -cr_w / 2 - CR_WALL_T
    eye = cbox(u0, cr_v0, beam_w - 3, CR_WALL_T, CLEAR - 1, 6)
    wall = cbox(
        u0, cr_v0, w_fork - 6, CR_WALL_T, cr_v1 - cr_v0, tt + CR_WALL_H - w_fork + 6
    )
    add(wall - eye, "Cradle – side wall", "#ffa000")
    add(
        eye,
        "Sensor: light beam across the cradle mouth – "
        + ("receiver" if sgn > 0 else "emitter"),
        "#212121",
    )
add(
    cbox(-cr_w / 2 + 6, cr_v0, w_fork - 1, 10, 7, 6),
    "Sensor: rod finder (reflective IR, looks at the rod ends)",
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
# The ToF sensor sits in a pocket in the pusher face and looks along the
# lane: a fetch stops at the fruit it actually sees (and an empty lane shows
# as the front stop); with a fruit on the fork it reads ~0, so it is also the
# check that the fruit is home against the face.
tof_w0 = tt + PUSHER_H - 14  # its underside
add(
    cbox(-fork_w / 2, push_face, w_chain, fork_w, PUSHER_T, tt + PUSHER_H - w_chain)
    - cbox(-7, push_face, tof_w0, 14, 2, 10),
    "Fork – pusher face",
    "#f4511e",
)
add(
    cbox(-7, push_face + 0.5, tof_w0, 14, 1.5, 10),
    "Sensor: ToF distance (VL53L1X), looks along the lane",
    "#212121",
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
dm_w, dm_d, dm_h = DRIVE_MOTOR
add(
    cbox(cheek_u + CHEEK_T, v_drive - dm_d / 2, w_drive - dm_h / 2, dm_w, dm_d, dm_h),
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
# Built in a (u, v, w) system like the carriage's at the scan dock, but with
# w = 0 at the roller tops: origin at the bed's rear edge, centred.
# Everything stands on one base plate with one connector (power + data), so
# the module lifts out of its drawer as a unit. A cable loop behind the
# drawer lets it pull out while connected.
# Put down: the fork carries the fruit in high, as for an insert, to 2 mm
#   short of the front stop, lowers it onto the rollers and retracts. The
#   tines pass inside the rollers, the keel between them, and both over the
#   rear support's crossbar, as in a lane.
# Spin: the rollers turn the fruit about its long axis in SPIN_STOPS steps.
#   At each stop the side camera images every LED band. Its distance is set
#   by its slide, per fruit or per run, and saved with the images. The near
#   roller hides the fruit's underside; the spin brings it into view.
# Pick up: as a fetch.
# Open questions for the bench: grip of the sleeves on the skin, how far a
# lopsided fruit walks along the rollers, and stray light from the customer
# section down the shaft.
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


w_base = -bed_im  # top of the base plate (the rear support stands on it)
u_sup = im_roll_u + im_roll_r  # the rollers' outer sides
add(
    mbox(u_mod0, v_mod0, w_base - MODULE_T, u_mod1 - u_mod0, -v_mod0, MODULE_T),
    "Imaging module – base plate",
    "#546e7a",
)
add(
    mbox(-u_sup, -XBAR_D, w_base, 2 * u_sup, XBAR_D, XBAR_H),
    "Imaging bed – rear support (crossbar under the fork)",
    "#78909c",
)
for sgn in (-1, 1):
    add(  # up to the roller's underside; its axle end runs in a bearing here
        mbox(
            sgn * im_roll_u - im_roll_r,
            -XBAR_D,
            w_base + XBAR_H,
            IM_ROLLER_D,
            XBAR_D,
            -IM_ROLLER_D - w_base - XBAR_H,
        ),
        "Imaging bed – rear roller bearing",
        "#78909c",
    )
    add(
        M
        * Pos(sgn * im_roll_u, v_stop / 2, -im_roll_r)
        * Rot(90, 0, 0)
        * Cylinder(im_roll_r, -v_stop),
        "Imaging bed – roller (driven, silicone sleeve)",
        "#90a4ae",
    )
add(
    mbox(u_bg, v_stop, w_base, 3, -v_stop, fruit_top_max_im + BG_UP - w_base),
    "Imaging bed – background panel (matte, also the far-side guard)",
    "#37474f",
)
add(  # on the panel, above the largest fruit
    mbox(u_bg - 1, scan_avo[0] - 20, fruit_top_max_im + 3, 1, 40, BG_UP - 6),
    "Reference tile (white / grey)",
    "#f5f5f5",
)
add(
    mbox(-u_sup, v_stop - STOP_T, w_base, 2 * u_sup, STOP_T, bed_im),
    "Imaging bed – front bearing bar",
    "#78909c",
)
add(
    mbox(-STOP_W / 2, v_stop - STOP_T, 0, STOP_W, STOP_T, BED_STOP_H),
    "Imaging bed – front stop",
    "#c0392b",
)
v_belt = v_stop - STOP_T - DRIVE_L
add(
    mbox(-u_sup, v_belt, w_base, u_drive + 28 + u_sup, DRIVE_L, bed_im + 8),
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
cam_bottom = scan_avo[1] - cw / 2  # the camera body's underside
add(
    mbox(-CAM_DIST - 12, scan_avo[0] - 12, w_base + 8, 24, 24, cam_bottom - w_base - 8),
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
        + (" (on rods)" if div_on_rod else " (in rod gaps)")
    )
    if LOAD_DRAWER:
        print(
            f" Loading gate:              {gate_z0:.0f}–{gate_z0 + gate_h:.0f}"
            f" mm over the rods (fruit centres {min(rests):.0f}–{max(rests):.0f}),"
            f" lifted {gate_rise:.0f} mm when closed"
        )
        if gate_top_up > tray_under_gate:
            print("   ! the lifted gate hits the top trays")
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
    print(f" Lift per fetch / insert:   {lift:.0f} mm  (elevator Z)")
    print(f" Fork reach (chain):        {fork_reach:.0f} mm  (rest -> front of a lane)")
    print(f" Cradle pocket (W x D):     {cr_w:.0f} x {push_rest - cr_v0:.0f} mm")
    print(f" Elevator shaft depth:      {elev_depth:.0f} mm")
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
        f" mm below the fork's dock height, {2 * car_half:.0f} mm wide"
    )
    print("=" * 56)
    print(" HOUSING AND TOP SECTION")
    print("=" * 56)
    print(
        f" Height at the back:        {h_back:.0f} mm  (front {H} mm; set by the chain"
        " magazine)"
    )
    print(
        f" Top trays:                 {top_slope_deg:.0f}°  (fruit slide from"
        f" {math.degrees(math.atan(TRAY_MU)):.0f}° at μ {TRAY_MU}), rear end"
        f" {top_rise:.0f} mm higher"
    )
    if div_back > H:
        print(f" Side walls over the trays: rise to {div_back:.0f} mm at the back")
    print("=" * 56)
    print(" IMAGING DRAWER AND MODULE")
    print("=" * 56)
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
        f" Imaging bed:               2 rollers Ø{IM_ROLLER_D} (sleeved), centres"
        f" {2 * im_roll_u:.0f} mm apart, {-v_stop:.0f} mm long, top z = {z_scan:.0f}"
    )
    print(
        f" Fork at the imaging bed:   tines {IM_ROLLER_CLEAR} mm inside the rollers,"
        f" docks {im_drop:.0f} mm below their tops, lift {lift_im:.0f} mm"
    )
    print(
        f" Spin:                      {across_min / IM_ROLLER_D:.1f}–"
        f"{across_max / IM_ROLLER_D:.1f} roller turns per fruit turn; a sideways"
        f" push of {hold_im:.2f} x its weight rolls a fruit off"
    )
    if hold_im < 0.5:
        print("   ! a fruit easily rolls off the rollers: set them wider apart")
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
    print(f" Level height:              {level_pitch:.0f} mm")
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
        need = H + N_SHELVES * level_pitch - avail_h
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
