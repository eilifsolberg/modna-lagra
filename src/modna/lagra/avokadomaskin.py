"""
Avokadomaskin - parametrisk konseptmodell (build123d)
=====================================================

Koordinatsystem (mm):
    X = bredde (venstre -> høyre)
    Y = dybde  (0 = forside/kunde, D = bakside/heis)
    Z = høyde  (0 = gulv)

Kjør:   python avokadomaskin.py
Output: avokadomaskin.step  (åpnes i Fusion, Onshape, FreeCAD ...)

Alle mål styres av parameterne under. Skriptet skriver også ut en
kapasitetsrapport, slik at du ser hva som faktisk får plass.
"""

import math
from build123d import (
    Box,
    Cylinder,
    Sphere,
    Compound,
    Location,
    Pos,
    Rot,
    Align,
    Color,
    scale,
    export_step,
)

# ---------------------------------------------------------------
# PARAMETERE
# ---------------------------------------------------------------
# Ytre mål
W, D, H = 600, 600, 800
WALL = 25  # vegg inkl. isolasjon

# Avokado (Hass, middels)
AVO_D = 70  # diameter
AVO_L = 100  # lengde
AVO_ROLLS = True  # True: langakse på tvers av raden (ruller)
# False: langakse langs raden (sklir)
CLEAR = 4  # klaring rundt avokado

# Vertikal inndeling
BASE_H = 120  # teknisk rom (kjøling/varme, elektronikk)
TOP_H = 130  # kundeseksjon på toppen
LOAD_LEVEL = True  # nederste nivå = påfyllingsrenne + skanner

# Hyller
N_SHELVES = 6  # ønsket antall (skriptet sjekker om det passer)
SHELF_T = 20  # hylletykkelse (isolert skille mellom soner)
SLOPE_DEG = 3.0  # helning ned mot heisen
FRONT_CLEAR = 10  # luft mellom hylle og forvegg
GATE_SPACE = 25  # plass under hylla til portmekanismen
FILL = 0.7  # andel av plassene som vises fylt

# Port (escapement) i bakkant av hver rad
GATE_TRAVEL = 19  # vertikal vandring for sperrene (mm)
GATE_FLAP_H = 17  # høyde på bakre sperre over hylleplata
GATE_TAB = 12  # hvor langt vippearmens rulle stikker inn i heissjakten

# Heis (bakside)
ELEV_DEPTH = 150  # dybde på heissjakten
ELEV_MODE = "hent"  # "hent":     vogna er dokket og slipper ut én avokado
# "legg_inn": skyveren dytter en avokado inn i raden
# "fri":      vogna står fritt ved ELEV_X / ELEV_Z
DOCK_SHELF = 2  # hylle vogna er dokket mot (1 = nederste lagerhylle)
DOCK_ROW = 2  # rad vogna er dokket mot (1 = venstre)
ELEV_X = 0.35  # kun for "fri" (0..1)
ELEV_Z = 0.55  # kun for "fri" (0..1)

# Kundeseksjon på toppen
TOP_SECTIONS = ["Myk moden", "Fast moden", "Moden om noen dager"]
TOP_SLOPE_DEG = 6.0  # heller mot kunden

# Temperatursoner nedenfra og opp (kun farge/etikett i modellen)
ZONE_TEMPS = [7, 10, 13, 16, 19, 22, 24, 26]

SHOW_HOUSING = True

# ---------------------------------------------------------------
# AVLEDEDE STØRRELSER
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
# HJELPEFUNKSJONER
# ---------------------------------------------------------------
parts = []


def add(shape, label, color):
    shape.label = label
    shape.color = Color(color)
    parts.append(shape)
    return shape


def box(x, y, z, dx, dy, dz):
    """Boks fra hjørne (x,y,z) med størrelse (dx,dy,dz)."""
    return Pos(x, y, z) * Box(dx, dy, dz, align=(Align.MIN, Align.MIN, Align.MIN))


def avocado(x, y, z, color, along_x=True):
    a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
    if not along_x:
        a = Rot(0, 0, 90) * a
    return add(Pos(x, y, z) * a, "Avokado", color)


def zone_color(i, n):
    # blå (kald) -> oransje (varm)
    t = i / max(n - 1, 1)
    r = 0.25 + 0.7 * t
    g = 0.45 + 0.1 * (1 - abs(0.5 - t) * 2)
    b = 0.85 - 0.65 * t
    return (r, g, b)


RIPE_COLORS = ["#2f3a1c", "#3f5a22", "#557a2b", "#6b9a33", "#86b83e", "#9fcf4c"]

# ---------------------------------------------------------------
# HUS
# ---------------------------------------------------------------
if SHOW_HOUSING:
    add(box(0, 0, 0, WALL, D, H), "Sidevegg V", "#d9d9d9")
    add(box(W - WALL, 0, 0, WALL, D, H), "Sidevegg H", "#d9d9d9")
    add(box(WALL, D - WALL, 0, inner_w, WALL, H - TOP_H), "Bakvegg", "#cfcfcf")
    add(box(WALL, 0, 0, inner_w, WALL, BASE_H), "Frontsokkel", "#bdbdbd")
add(
    box(WALL, WALL, 0, inner_w, D - 2 * WALL, BASE_H - 5),
    "Teknisk rom (kjøl/varme, styring)",
    "#7a7a7a",
)

# ---------------------------------------------------------------
# PÅFYLLINGSNIVÅ: renne forfra -> skanner -> heis
# ---------------------------------------------------------------
z = storage_z0
if LOAD_LEVEL:
    chute_w = row_pitch * 1.3
    chute_len = shelf_y1 - WALL
    chute_top = z + chute_len * math.tan(slope) + 5
    chute = box(0, 0, -SHELF_T / 2, chute_w, chute_len, SHELF_T / 2)
    chute = Pos(W / 2 - chute_w / 2, WALL, chute_top) * Rot(-SLOPE_DEG, 0, 0) * chute
    add(chute, "Påfyllingsrenne", "#b0b0b0")
    # innkast-åpning i front (ramme)
    add(box(W / 2 - 80, 0, z, 160, WALL, 10), "Innkast – ramme nede", "#444444")
    add(
        box(W / 2 - 80, 0, z + AVO_D + 30, 160, WALL, 10),
        "Innkast – ramme oppe",
        "#444444",
    )
    # multispektralt kamera over renna, nær heisen
    cam_y = shelf_y1 - 190
    cam_z = z + AVO_D + 12  # plassert rett under hylle 1
    add(box(W / 2 - 45, cam_y, cam_z, 90, 70, 16), "Multispektralt kamera", "#1f1f1f")
    add(Pos(W / 2, cam_y + 35, cam_z - 3) * Cylinder(15, 6), "Kameralinse", "#3050a0")
    avocado(
        W / 2,
        cam_y + 35,
        chute_top - (cam_y + 35 - WALL) * math.tan(slope) + AVO_D / 2,
        "#557a2b",
        along_x=AVO_ROLLS,
    )
    z += level_pitch

# ---------------------------------------------------------------
# LAGRINGSHYLLER MED PORTMEKANISME
# ---------------------------------------------------------------
# Hver hylle bygges i et lokalt koordinatsystem som følger hylleplata:
#   lx = 0..inner_w (bredde), ly = 0 foran .. shelf_len bak (heis),
#   lz = 0 på hylleplatas overflate. Deretter vippes alt SLOPE_DEG.
#
# Portmekanisme per rad (passiv, ingen motor per rad):
#   - En vippearm under hylla dreier om en aksel.
#   - Bak: en sperre (stolpe + fjærbelastet klaff) holder køen.
#   - Foran: en pinne som ligger senket i hvilestilling.
#   - En sidearm med rulle stikker GATE_TAB mm inn i heissjakten.
#   Når heisvogna trykker rullen ned, senkes bakre sperre og fremre
#   pinne løftes mellom avokado 1 og 2: kun én avokado slipper ut.
#   Klaffen på bakre sperre kan felles innover, slik at skyveren kan
#   dytte en ny avokado inn i raden uten å åpne porten (LIFO).

rows_w = n_rows * row_pitch
x_off = WALL + (inner_w - rows_w) / 2
a_sl = -slope  # vinkel for lokal -> global

y_rear = shelf_len - 4  # bakre sperre (senter)
y_front = shelf_len - 6 - slot_len  # fremre pinne (mellom avokado 1 og 2)
y_piv = (y_rear + y_front) / 2
d_arm = (y_rear - y_front) / 2
z_piv = -SHELF_T - 14
phi0 = math.asin(GATE_TRAVEL / (2 * d_arm))  # ± vippevinkel
L_tab = shelf_len + GATE_TAB - y_piv  # aksel -> rulle
rear_len = GATE_FLAP_H - (z_piv + d_arm * math.sin(phi0))
front_len = -2 - (z_piv - d_arm * math.sin(phi0))
TAB_X = row_pitch / 2 - 12  # rullens sideforskyvning fra radsenter


def to_world(base, lx, ly, lz):
    """Lokalt hyllepunkt -> globale koordinater."""
    return (
        WALL + lx,
        shelf_y0 + ly * math.cos(a_sl) - lz * math.sin(a_sl),
        base + ly * math.sin(a_sl) + lz * math.cos(a_sl),
    )


def row_cx(r):
    return x_off - WALL + r * row_pitch + row_pitch / 2


def tab_roller_z(phi):
    """Rullens senterhøyde (lokalt) for en gitt vippevinkel."""
    return z_piv + L_tab * math.sin(phi)


def gate(cxl, released=False, flap_folded=False):
    """Portmekanisme for én rad, i hyllas lokale system."""
    phi = -phi0 if released else phi0
    out = []
    # Vippearm (roterer om akselen)
    rocker = [
        (Rot(0, 90, 0) * Cylinder(4, 100), "Port – aksel", "#9e9e9e"),
        (box(-4, -d_arm - 8, -3, 8, 2 * d_arm + 16, 6), "Port – vippearm", "#546e7a"),
        (box(TAB_X - 4, -6, -3, 8, L_tab + 6, 6), "Port – sidearm", "#546e7a"),
        (
            Pos(TAB_X, L_tab, 0) * Rot(0, 90, 0) * Cylinder(5, 12),
            "Port – rulle",
            "#ffca28",
        ),
    ]
    T = Pos(cxl, y_piv, z_piv) * Rot(math.degrees(phi), 0, 0)
    out += [(T * shp, lab, col) for shp, lab, col in rocker]

    # Bakre sperre: stolpe + klaff
    z_r = z_piv + d_arm * math.sin(phi)
    top_r = z_r + rear_len
    hinge_z = top_r - GATE_FLAP_H - 1
    out.append(
        (
            box(cxl - 10, y_rear - 2.5, z_r, 20, 5, hinge_z - z_r),
            "Port – bakre stolpe",
            "#c0392b",
        )
    )
    out.append(
        (
            Pos(cxl, y_rear, hinge_z) * Rot(0, 90, 0) * Cylinder(3, 36),
            "Port – klaffhengsel",
            "#7f8c8d",
        )
    )
    flap = box(-18, -2, 0, 36, 4, GATE_FLAP_H + 1)
    fold = 80 if flap_folded else 0
    out.append(
        (
            Pos(cxl, y_rear, hinge_z) * Rot(fold, 0, 0) * flap,
            "Port – klaff (enveis)",
            "#e74c3c",
        )
    )

    # Fremre pinne
    z_f = z_piv - d_arm * math.sin(phi)
    out.append(
        (
            Pos(cxl, y_front, z_f)
            * Cylinder(4, front_len, align=(Align.CENTER, Align.CENTER, Align.MIN)),
            "Port – fremre pinne",
            "#c0392b",
        )
    )
    out.append(
        (Pos(cxl, y_front, z_f + front_len) * Sphere(4), "Port – pinnetopp", "#c0392b")
    )

    # Faste braketter under hylla
    for sx in (-50, 26):
        out.append(
            (
                box(cxl + sx, y_piv - 8, z_piv - 8, 4, 16, -SHELF_T - (z_piv - 8)),
                "Port – brakett",
                "#78909c",
            )
        )
    return out


docked = ELEV_MODE in ("hent", "legg_inn")
dock_origin = None
shelf_bases = []

for s in range(n_shelves):
    zc = zone_color(s, n_shelves)
    temp = ZONE_TEMPS[s] if s < len(ZONE_TEMPS) else "?"
    base = z + drop  # høyeste punkt (front)
    shelf_bases.append(base)
    T = Pos(WALL, shelf_y0, base) * Rot(-SLOPE_DEG, 0, 0)
    local = []

    # Hylleplate med utsparinger for sperrene
    plate = box(0, 0, -SHELF_T, inner_w, shelf_len, SHELF_T)
    for r in range(n_rows):
        cxl = row_cx(r)
        plate -= box(cxl - 20, y_rear - 5, -SHELF_T - 1, 40, 10, SHELF_T + 2)
        plate -= Pos(cxl, y_front, -SHELF_T / 2) * Cylinder(6, SHELF_T + 2)
    local.append((plate, f"Hylle {s + 1} ({temp} °C)", zc))

    # Radskillere
    for r in range(n_rows + 1):
        rx = x_off - WALL + r * row_pitch - 2
        local.append((box(rx, 0, 0, 4, shelf_len, 35), "Radskiller", "#eeeeee"))

    # Porter + avokadoer
    n_fill = round(per_row * FILL)
    for r in range(n_rows):
        cxl = row_cx(r)
        is_dock = docked and s == DOCK_SHELF - 1 and r == DOCK_ROW - 1
        local += gate(
            cxl,
            released=is_dock and ELEV_MODE == "hent",
            flap_folded=is_dock and ELEV_MODE == "legg_inn",
        )
        col = RIPE_COLORS[(s + r) % len(RIPE_COLORS)]
        k0, shift = 0, 0
        if is_dock and ELEV_MODE == "hent":
            k0 = 1  # avokado 1 har rullet over i vugga
            dock_origin = to_world(base, cxl, shelf_len, 0)
        if is_dock and ELEV_MODE == "legg_inn":
            shift = 40  # køen er dyttet ett hakk fremover
            dock_origin = to_world(base, cxl, shelf_len, 0)
        for k in range(k0, n_fill):
            cy = shelf_len - 6 - slot_len / 2 - k * slot_len - shift
            a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
            if not AVO_ROLLS:
                a = Rot(0, 0, 90) * a
            local.append((Pos(cxl, cy, AVO_D / 2 + 1) * a, "Avokado", col))

    for shp, lab, col in local:
        add(T * shp, lab, col)

    z += level_pitch

# ---------------------------------------------------------------
# KUNDESEKSJON PÅ TOPPEN
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
    add(tray, f"Toppseksjon: {name}", "#8d6e63")
    # lav kant foran + skiller
    add(
        box(sx + 3, WALL - 10, top_z - 12, sec_w - 6, 10, 45),
        f"Frontkant {name}",
        "#5d4037",
    )
    div = Pos(sx, WALL, top_z) * Rot(TOP_SLOPE_DEG, 0, 0) * box(0, 0, 0, 6, top_len, 40)
    add(div, "Skiller topp", "#5d4037")
    # avokadoer samlet foran (ruller mot kunden)
    n_side = int((sec_w - 20) // (AVO_D + 5))
    for rr in range(2):
        for c in range(n_side):
            ax = sx + 10 + (AVO_D + 5) / 2 + c * (AVO_D + 5)
            ay = WALL + AVO_L / 2 + 5 + rr * (AVO_L + 5)
            az = top_z + (ay - WALL) * math.tan(top_slope) + AVO_D / 2
            avocado(ax, ay, az, top_colors[i], along_x=False)

if SHOW_HOUSING:
    # topplate bak (dekker heisens toppstasjon)
    add(
        box(WALL, D - WALL - ELEV_DEPTH, H - 20, inner_w, ELEV_DEPTH + WALL, 20),
        "Topplate over heis",
        "#cfcfcf",
    )

# ---------------------------------------------------------------
# HEIS (X-Z-portal på baksiden) MED DETALJERT VUGGE
# ---------------------------------------------------------------
# Vugga bygges i et eget system (u, v, w) med origo i hyllas bakkant
# (radsenter, overflate) når vogna er dokket:
#   u = sideveis (X), v = bakover inn i sjakta (Y), w = opp (Z)
#
# Hente (ELEV_MODE = "hent"):
#   1. Vogna stiller seg ~27 mm over dokkhøyden ved riktig rad.
#   2. Tunga skyves frem over portens rulle (liten solenoid).
#   3. Vogna senker seg: tunga presser rullen ned -> porten åpner,
#      avokado 1 ruller over i vugga, pinnen holder resten av køen.
#   4. Vogna løfter seg, tunga trekkes inn, fjæren lukker porten.
# Legge inn (ELEV_MODE = "legg_inn"):
#   Porten er lukket. Skyveren dytter avokadoen forbi den fjærbelastede
#   klaffen og skyver hele køen ett hakk opp bakken. Klaffen smetter
#   opp bak avokadoen og holder den på plass.
#   Samme skyver leverer også til kundeseksjonene på toppen.

x_travel0 = WALL + 30
x_travel1 = W - WALL - 30
z_travel0 = storage_z0
z_travel1 = H - 40
SHAFT = D - WALL - shelf_y1  # sjaktdybde i v-retning

if dock_origin is None:  # "fri" eller ugyldig dokkposisjon
    dock_origin = (
        x_travel0 + ELEV_X * (x_travel1 - x_travel0),
        shelf_y1,
        z_travel0 + ELEV_Z * (z_travel1 - z_travel0),
    )
ox, oy, oz = dock_origin
C = Pos(ox, oy, oz)


def cbox(u, v, w, du, dv, dw):
    return C * box(u, v, w, du, dv, dw)


# Portal: X-skinner nede og oppe, Z-søyle med lineærskinne
for zr in (z_travel0 - 30, z_travel1):
    add(box(WALL, D - WALL - 24, zr, inner_w, 24, 18), "X-skinne", "#607d8b")
col_z0, col_z1 = z_travel0 - 12, z_travel1
add(
    box(ox - 20, oy + 126, col_z0, 40, SHAFT - 126, col_z1 - col_z0),
    "Z-aktuator (søyle)",
    "#455a64",
)
add(box(ox - 8, oy + 120, col_z0, 16, 6, col_z1 - col_z0), "Z-lineærskinne", "#b0bec5")

# Vogn
add(cbox(-55, 92, -55, 110, 16, 155), "Vognplate", "#37474f")
for w0 in (-45, 55):
    add(cbox(-22, 108, w0, 44, 12, 30), "Lagerblokk", "#263238")

# Vugge: bunn og sidevegger (innvendig bredde > avokadoens lengde)
CR_W = (AVO_L if AVO_ROLLS else AVO_D) + 9
add(cbox(-CR_W / 2, 16, -13, CR_W, 76, 5), "Vugge – bunn", "#ffb300")
for sgn in (-1, 1):
    u0 = CR_W / 2 if sgn > 0 else -CR_W / 2 - 4
    add(cbox(u0, 16, -13, 4, 76, 50), "Vugge – sidevegg", "#ffa000")
add(cbox(-CR_W / 2 + 2, 60, 8, 8, 14, 10), "Sensor: avokado i vugga", "#212121")

# Tunge + solenoid som betjener portens rulle
tongue_top = tab_roller_z(-phi0) + 5  # rullens topp når porten er åpen
if ELEV_MODE == "hent":
    tv0 = GATE_TAB - 8  # tunga fremme, over rulla
else:
    tv0 = 24  # tunga trukket inn
add(cbox(TAB_X - 7, tv0, tongue_top, 14, 30, 5), "Tunge", "#e65100")
add(
    cbox(TAB_X - 12, 30, tongue_top - 4, 24, 40, -13 - (tongue_top - 4)),
    "Solenoid for tunge",
    "#424242",
)

# Skyver drevet av en liten lineæraktuator over vugga
push_face = 76 if ELEV_MODE != "legg_inn" else 30
add(cbox(-40, push_face, 2, 80, 5, 56), "Skyverplate", "#f4511e")
add(cbox(-6, push_face + 5, 50, 12, 8, 30), "Skyverarm", "#bf360c")
add(cbox(-14, push_face + 2, 78, 28, 16, 4), "Skyver – slede", "#90a4ae")
add(cbox(-18, 20, 82, 36, 90, 18), "Lineæraktuator for skyver", "#546e7a")
add(cbox(-18, 104, 82, 36, 6, 18), "Aktuator-feste", "#37474f")

# Avokado i vugga
a = scale(Sphere(1), by=(AVO_L / 2, AVO_D / 2, AVO_D / 2))
if not AVO_ROLLS:
    a = Rot(0, 0, 90) * a
if ELEV_MODE == "legg_inn":
    add(C * Pos(0, push_face - AVO_D / 2, AVO_D / 2 + 4) * a, "Avokado", "#6b9a33")
else:
    add(C * Pos(0, push_face - AVO_D / 2, -8 + AVO_D / 2) * a, "Avokado", "#6b9a33")

# ---------------------------------------------------------------
# EKSPORT + RAPPORT
# ---------------------------------------------------------------
model = Compound(children=parts, label="Avokadomaskin")

if __name__ == "__main__":
    export_step(model, "avokadomaskin.step")

    print("=" * 56)
    print(" PORT OG HEIS")
    print("=" * 56)
    print(f" Vippevinkel porter:        ±{math.degrees(phi0):.1f}°")
    print(
        f" Dokk-senkning (tunge):     {tab_roller_z(phi0) - tab_roller_z(-phi0):.0f} mm"
    )
    print(f" Skyverslag inn i rad:      {76 - (-6):.0f} mm")
    print(f" Vugge innvendig bredde:    {CR_W:.0f} mm")
    print("=" * 56)
    print(" KAPASITETSRAPPORT")
    print("=" * 56)
    print(f" Hylleflate (B x D):        {inner_w:.0f} x {shelf_len:.0f} mm")
    print(f" Rader per hylle:           {n_rows}  (radbredde {row_pitch:.0f} mm)")
    print(f" Avokado per rad:           {per_row}")
    print(f" Avokado per hylle:         {n_rows * per_row}")
    print(f" Nivåhøyde (inkl. helning): {level_pitch:.0f} mm")
    print(
        f" Nivåer som får plass:      {n_levels_fit}"
        + ("  (1 brukt til påfylling)" if LOAD_LEVEL else "")
    )
    print(f" Lagringshyller i modellen: {n_shelves}  (ønsket {N_SHELVES})")
    print(f" Total lagerkapasitet:      {n_shelves * n_rows * per_row}")
    if n_shelves < N_SHELVES:
        need = (
            (N_SHELVES + (1 if LOAD_LEVEL else 0)) * level_pitch + drop + BASE_H + TOP_H
        )
        print(f" ! {N_SHELVES} hyller krever ca. {need:.0f} mm total høyde.")
    print("=" * 56)

    # -----------------------------------------------------------
    # VIS I VS CODE (OCP CAD Viewer)
    # -----------------------------------------------------------
    # Krever utvidelsen «OCP CAD Viewer» i VS Code og:
    #   pip install ocp_vscode
    # Viewer-panelet må være åpent (klikk OCP-ikonet i VS Code).
    # Delene vises med navn og farger, og kan skjules i trestrukturen.
    try:
        from ocp_vscode import show, set_defaults, Camera

        set_defaults(reset_camera=Camera.KEEP)  # behold kameravinkel ved ny kjøring
        show(model)
    except ImportError:
        print(" (ocp_vscode ikke installert – hopper over 3D-visning)")
    except Exception as e:  # f.eks. viewer-panelet er ikke åpent
        print(f" (Kunne ikke vise i OCP CAD Viewer: {e})")
