# TritonFishingBoats

## CC35 stepped-V hull (Grasshopper) — current stage (2026-03-16)

This repo currently contains work-in-progress hull generation for a ~35 ft (10.67 m) offshore center console with twin 275s and a 2-step bottom. The current implementation is a **single Grasshopper Python (Rhino 8 / Python 3) component** driven by sliders.

### What’s implemented

- Section-based hull generation per station (port + mirrored starboard)
- Variable deadrise (deep forward, moderate aft)
- 2 strakes per side (modeled as section breakpoints)
- Reversed chine shelf (simple) for stability + spray control
- 2-step bottom (panels split into aft / mid / fwd regions to avoid loft twist)
- Stem closure station (bow closes)
- Guide curves output (sheer, chine, keel) for inspection
- Debug output (computed effective beam from generated sheer/chine)

### Grasshopper setup

1. In Rhino 8, open **Grasshopper**.
2. Drop a **Python 3 Script** component.
3. Create number sliders for the inputs listed below and wire them to the Python component.
4. Rename Python component outputs to:
   - `BottomBreps`
   - `SideBreps`
   - `Guides`
   - `Debug`

### Inputs (sliders)

Recommended starting values (units meters / degrees):

- `LOA` = `10.67`
- `BEAM` = `3.20`
- `N` = `81` (integer)

**Steps**
- `STEP1_RATIO` = `0.40`
- `STEP2_RATIO` = `0.60`
- `STEP_H` = `0.045`
- `STEP_CHINE_FACTOR` = `0.28`

**Deadrise**
- `DEAD_AFT` = `22.0`
- `DEAD_FWD` = `48.0`
- `DEAD_PWR` = `1.25`

**Sheer / flare**
- `FREE_T` = `0.95`
- `FREE_BOW_ADD` = `0.70`
- `FLARE` = `0.65`
- `FWD_FLARE_MULT` = `1.40`
- `FWD_FLARE_START` = `0.55`

**Beam taper**
- `BOW_END_SCALE` = `0.30`
- `BOW_TAPER_P` = `2.2`
- `BOW_MIN_HB` = `0.12`

**Chine shelf**
- `SHELF_MAX` = `0.10`
- `SHELF_START` = `0.35`
- `SHELF_ZDROP` = `0.03`
- `CHINE_MIN_HB` = `0.10`

**Strakes**
- `S1_FRAC` = `0.55`
- `S2_FRAC` = `0.75`
- `S1_Z` = `0.03`
- `S2_Z` = `0.05`

**Stem closure**
- `STEM_EXT` = `0.08`
- `STEM_DROP` = `0.00`

### GhPython code (current)

Paste this into the Python 3 component.

```python
import math
import Rhino
import Rhino.Geometry as rg

tol = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance

def clamp(x,a,b):
    return a if x < a else (b if x > b else x)

def lerp(a,b,t):
    return a + (b-a)*t

def remap01(x,a,b):
    if abs(b-a) < 1e-9:
        return 0.0
    return clamp((x-a)/(b-a), 0.0, 1.0)

def smoothstep(t):
    t = clamp(t,0.0,1.0)
    return t*t*(3.0-2.0*t)

def mirror_y(pt):
    return rg.Point3d(pt.X, -pt.Y, pt.Z)

def loft(curves):
    if len(curves) < 2:
        return None
    breps = rg.Brep.CreateFromLoft(curves, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Normal, False)
    if breps and len(breps) > 0:
        return breps[0]
    return None

# --- inputs ---
N = max(9, int(N))
HB_target = BEAM * 0.5

x1 = LOA * STEP1_RATIO
x2 = LOA * STEP2_RATIO

# First pass: compute points per station (unscaled Y)
stations = []

for i in range(N):
    t = i / float(N-1)
    x = LOA * t

    # step offset
    if x < x1:
        dz = 0.0
        region = 0
    elif x < x2:
        dz = STEP_H
        region = 1
    else:
        dz = 2.0 * STEP_H
        region = 2

    # baseline half-beam (before flare)
    beam_scale = (1.0 - (t ** BOW_TAPER_P)) * (1.0 - BOW_END_SCALE) + BOW_END_SCALE
    half_beam = max(BOW_MIN_HB, HB_target * beam_scale)

    bow_zone = smoothstep((t - 0.80) / 0.20)

    # forward flare multiplier
    fwd = remap01(t, FWD_FLARE_START, 1.0)
    fwd_flare_gain = 1.0 + (FWD_FLARE_MULT - 1.0) * (fwd ** 1.3)

    # sheer
    z_sheer = FREE_T + FREE_BOW_ADD * (t ** 1.30) + 0.10 * bow_zone
    flare_gain = (0.10 + 0.55 * (t ** 1.6)) * FLARE * fwd_flare_gain
    y_sheer = half_beam * (1.0 + flare_gain)

    # chine + shelf
    chine_inboard = 0.06 + 0.09 * (t ** 1.15)
    y_chine_base = half_beam * (0.92 - chine_inboard)
    shelf_t = remap01(t, SHELF_START, 1.0)
    y_chine = y_chine_base + SHELF_MAX * (shelf_t ** 1.2)
    y_chine = max(CHINE_MIN_HB, y_chine)

    z_chine = lerp(0.25, 0.62, t ** 1.10) + STEP_CHINE_FACTOR * dz
    z_chine_shelf = z_chine - SHELF_ZDROP * (shelf_t ** 1.15)

    # deadrise / keel
    deadrise_deg = lerp(DEAD_AFT, DEAD_FWD, t ** DEAD_PWR)
    deadrise = math.radians(deadrise_deg)
    vdrop = math.tan(deadrise) * y_chine

    keel_base = -lerp(1.35, 0.55, t ** 1.35) + 0.18 * bow_zone
    z_keel_from_chine = z_chine_shelf - vdrop
    z_keel = lerp(keel_base, z_keel_from_chine, 0.78) + dz

    # strakes
    y_s1 = y_chine * S1_FRAC
    z_s1 = lerp(z_keel, z_chine_shelf, S1_FRAC) + S1_Z * (0.30 + 0.70 * shelf_t)

    y_s2 = y_chine * S2_FRAC
    z_s2 = lerp(z_keel, z_chine_shelf, S2_FRAC) + S2_Z * (0.30 + 0.70 * shelf_t)

    stations.append({
        "region": region,
        "x": x,
        "keel": (0.0, z_keel),
        "s1": (y_s1, z_s1),
        "s2": (y_s2, z_s2),
        "chine": (y_chine, z_chine_shelf),
        "sheer": (y_sheer, z_sheer),
    })

# Beam lock: scale all Y so max sheer = BEAM/2
max_y_sheer = max(abs(s["sheer"][0]) for s in stations)
scale_y = (HB_target / max_y_sheer) if max_y_sheer > 1e-9 else 1.0

# Second pass: build curves and breps using scaled Y
bot_port = []
bot_star = []
side_port = []
side_star = []

sheer_pts = []
chine_pts = []
keel_pts = []

for s in stations:
    x = s["x"]
    region = s["region"]

    P_keel = rg.Point3d(x, 0.0, s["keel"][1])
    P_s1   = rg.Point3d(x, s["s1"][0] * scale_y, s["s1"][1])
    P_s2   = rg.Point3d(x, s["s2"][0] * scale_y, s["s2"][1])
    P_chine= rg.Point3d(x, s["chine"][0] * scale_y, s["chine"][1])
    P_sheer= rg.Point3d(x, s["sheer"][0] * scale_y, s["sheer"][1])

    crv_bot_p = rg.PolylineCurve([P_keel, P_s1, P_s2, P_chine])
    crv_side_p= rg.PolylineCurve([P_chine, P_sheer])

    crv_bot_s = rg.PolylineCurve([mirror_y(P_keel), mirror_y(P_s1), mirror_y(P_s2), mirror_y(P_chine)])
    crv_side_s= rg.PolylineCurve([mirror_y(P_chine), mirror_y(P_sheer)])

    bot_port.append((region, crv_bot_p))
    bot_star.append((region, crv_bot_s))
    side_port.append((region, crv_side_p))
    side_star.append((region, crv_side_s))

    sheer_pts.append(P_sheer)
    chine_pts.append(P_chine)
    keel_pts.append(P_keel)

# Stem closure station (centerline)
x_stem = LOA + STEM_EXT
P_sheer_bow = sheer_pts[-1]
P_chine_bow = chine_pts[-1]
P_keel_bow  = keel_pts[-1]

P_keel_stem  = rg.Point3d(x_stem, 0.0, P_keel_bow.Z - STEM_DROP)
P_chine_stem = rg.Point3d(x_stem, 0.0, P_chine_bow.Z)
P_sheer_stem = rg.Point3d(x_stem, 0.0, P_sheer_bow.Z)

crv_bot_stem  = rg.PolylineCurve([P_keel_stem, P_chine_stem])
crv_side_stem = rg.PolylineCurve([P_chine_stem, P_sheer_stem])

bot_port.append((2, crv_bot_stem)); bot_star.append((2, crv_bot_stem))
side_port.append((2, crv_side_stem)); side_star.append((2, crv_side_stem))

# Guides
Guides = [
    rg.Curve.CreateInterpolatedCurve(sheer_pts, 3),
    rg.Curve.CreateInterpolatedCurve([mirror_y(p) for p in sheer_pts], 3),
    rg.Curve.CreateInterpolatedCurve(chine_pts, 3),
    rg.Curve.CreateInterpolatedCurve([mirror_y(p) for p in chine_pts], 3),
    rg.Curve.CreateInterpolatedCurve(keel_pts, 3),
]

def curves_for_region(tagged, region):
    return [c for (r,c) in tagged if r == region]

BottomBreps = []
SideBreps = []

for reg in (0,1,2):
    bp = curves_for_region(bot_port, reg)
    bs = curves_for_region(bot_star, reg)
    sp = curves_for_region(side_port, reg)
    ss = curves_for_region(side_star, reg)

    b1 = loft(bp); b2 = loft(bs); s1 = loft(sp); s2 = loft(ss)
    if b1: BottomBreps.append(b1)
    if b2: BottomBreps.append(b2)
    if s1: SideBreps.append(s1)
    if s2: SideBreps.append(s2)

# Transom cap
P_keel_T  = keel_pts[0]
P_chine_T = chine_pts[0]
P_sheer_T = sheer_pts[0]
P_keel_Ts  = mirror_y(P_keel_T)
P_chine_Ts = mirror_y(P_chine_T)
P_sheer_Ts = mirror_y(P_sheer_T)

transom_poly = rg.Polyline([P_keel_T, P_chine_T, P_sheer_T, P_sheer_Ts, P_chine_Ts, P_keel_Ts, P_keel_T])
transom_crv = rg.PolylineCurve(transom_poly)

caps = rg.Brep.CreatePlanarBreps(transom_crv, tol)
if caps and len(caps) > 0:
    SideBreps.append(caps[0])

max_sheer_y2 = max(abs(p.Y) for p in sheer_pts) if sheer_pts else 0.0
max_chine_y2 = max(abs(p.Y) for p in chine_pts) if chine_pts else 0.0
Debug = "Beam-locked: scaleY=%.4f | max|Y| sheer=%.3f (beam=%.3f), chine=%.3f (beam=%.3f)" % (
    scale_y, max_sheer_y2, 2*max_sheer_y2, max_chine_y2, 2*max_chine_y2
) 
``` 

### Next tweaks planned

- Add bow-shape sliders (stem sharpness/rounding, entry, flare distribution)
- Add explicit step faces (vertical surfaces) and better step edge control
- Optional: add a short aft “flat run” region for a more production-like planing surface
