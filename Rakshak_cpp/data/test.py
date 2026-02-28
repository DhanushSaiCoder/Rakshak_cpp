# -*- coding: utf-8 -*-
# ============================================================
#  Zoom-aware interpolation tests using PERSON_DATA
#  Rank (best -> worst for bbox-height → depth):
#    1) Isotonic Regression → PCHIP
#    2) PCHIP
#    3) Akima
#    4) Linear
#    5) Univariate Spline
#    6) Isotonic (stepwise)
# ============================================================

import numpy as np

# ------------------------------------------------------------
# PERSON_DATA must map zoom levels as top-level keys inside
# PERSON_DATA["person_data"], e.g. "1", "2", ..., "30".
# If you already defined it earlier, remove this stub.
# ------------------------------------------------------------
PERSON_DATA = {
    "person_data": {
        "1": {"depth":[5.0,9.8,10.0,15.0,20.0,25.0,30.0,35.0,40.0],
              "x":[923,718,884,921,971,1005,1009,1011,1015],
              "y":[658,593,596,585,565,556,550,551,546],
              "w":[136,98,90,55,40,41,34,29,24],
              "h":[462,275,280,186,132,105,91,80,67]},
        "2": {"depth":[10.0,20.0,30.0,40.0,50.0,60.0,70.0],
              "x":[1252,966,1162,1146,1052,972,950],
              "y":[649,608,572,566,557,546,542],
              "w":[182,90,68,45,42,32,24],
              "h":[528,260,177,136,108,88,76]},
        "3": {"depth":[15.0,30.0,45.0,60.0,75.0,90.0,110.0,110.0],
              "x":[1469,1257,1039,966,910,975,926,922],
              "y":[675,599,581,556,548,542,540,539],
              "w":[204,91,66,52,42,32,35,27],
              "h":[581,265,176,131,106,86,76,75]},
        "4": {"depth":[20.0,40.0,60.0,80.0,100.0,125.0,150.0,175.0],
              "x":[1205,1030,852,976,914,903,966,920],
              "y":[675,616,579,551,548,541,540,532],
              "w":[154,86,65,54,35,33,24,23],
              "h":[517,265,183,138,113,83,70,67]},
        "5": {"depth":[25.0,50.0,50.0,75.0,100.0,125.0,150.0],
              "x":[1517,1081,890,859,905,888,957],
              "y":[684,611,616,563,549,545,541],
              "w":[158,94,89,62,53,41,44],
              "h":[522,270,268,183,116,107,85]},
        "6": {"depth":[30.0,60.0,90.0,150.0,200.0],
              "x":[847,1154,987,967,955],
              "y":[691,588,563,546,534],
              "w":[177,88,61,35,32],
              "h":[522,269,174,101,81]},
        "7": {"depth":[35.0,70.0,110.0,150.0,175.0],
              "x":[1610,897,853,961,878],
              "y":[695,590,557,554,542],
              "w":[184,86,62,42,42],
              "h":[547,267,169,126,104]},
        "8": {"depth":[40.0,80.0,125.0,175.0,200.0],
              "x":[1101,995,834,870,952],
              "y":[712,579,563,543,541],
              "w":[155,94,80,43,34],
              "h":[526,275,173,125,104]},
        "9": {"depth":[45.0,90.0,150.0,200.0],
              "x":[876,984,957,948],
              "y":[706,584,562,541],
              "w":[179,92,51,39],
              "h":[538,264,158,124]},
        "10":{"depth":[50.0,100.0,100.0,150.0],
              "x":[806,829,835,952],
              "y":[713,589,585,565],
              "w":[163,101,108,62],
              "h":[545,258,268,176]},
        "11":{"depth":[55.0,110.0,175.0],
              "x":[1117,787,820],
              "y":[695,579,552],
              "w":[280,90,68],
              "h":[542,264,171]},
        "12":{"depth":[60.0,125.0,200.0],
              "x":[1320,774,934],
              "y":[657,584,552],
              "w":[178,90,64],
              "h":[536,256,163]},
        "13":{"depth":[65.0,150.0,200.0],
              "x":[961,948,947],
              "y":[675,579,552],
              "w":[168,74,61],
              "h":[517,232,168]},
        "14":{"depth":[70.0,150.0,200.0],
              "x":[490,943,930],
              "y":[682,583,558],
              "w":[175,87,66],
              "h":[511,251,186]},
        "15":{"depth":[75.0,150.0],
              "x":[1184,940],
              "y":[637,588],
              "w":[187,94],
              "h":[531,268]},
        "16":{"depth":[80.0,85.0,175.0,200.0],
              "x":[1244,760,747,935],
              "y":[636,643,573,566],
              "w":[195,168,88,61],
              "h":[535,509,242,205]},
        "17":{"depth":[85.0,175.0],
              "x":[848,738],
              "y":[643,576],
              "w":[183,88],
              "h":[533,264]},
        "18":{"depth":[90.0,200.0],
              "x":[1054,911],
              "y":[643,563],
              "w":[230,94],
              "h":[527,238]},
        "19":{"depth":[95.0,200.0],
              "x":[765,916],
              "y":[660,570],
              "w":[154,85],
              "h":[530,253]},
        "20":{"depth":[100.0,110.0,200.0],
              "x":[1062,585,930],
              "y":[652,630,576],
              "w":[179,147,95],
              "h":[523,485,263]},
        "21":{"depth":[110.0,250.0,300.0,350.0,400.0,450.0,500.0],
              "x":[622,1037,727,1083,857,990,857],
              "y":[638,396,386,527,579,568,732],
              "w":[166,67,68,59,43,39,41],
              "h":[510,224,185,160,141,121,115]},
        "22":{"depth":[110.0,200.0,250.0,300.0,350.0,400.0,450.0,500.0],
              "x":[603,907,1043,709,1101,852,992,855],
              "y":[645,579,391,380,529,583,570,746],
              "w":[175,99,68,67,60,46,41,44],
              "h":[535,295,236,190,172,150,127,120]},
        "23":{"depth":[125.0,300.0,350.0,400.0,450.0,500.0],
              "x":[597,700,1105,849,994,850],
              "y":[645,374,529,583,572,756],
              "w":[153,70,63,46,49,45],
              "h":[492,204,177,150,132,126]},
        "24":{"depth":[250.0,300.0,350.0,400.0,450.0,500.0],
              "x":[1046,689,1112,845,995,845],
              "y":[379,369,530,586,579,765],
              "w":[86,69,60,67,49,48],
              "h":[257,214,185,154,130,125]},
        "25":{"depth":[125.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[576,1051,676,1118,836,1001,838,808],
              "y":[659,372,364,528,589,579,773,786],
              "w":[229,88,73,59,59,51,48,49],
              "h":[530,263,223,191,163,137,131,128]},
        "26":{"depth":[150.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[925,1057,667,1123,834,998,834,802],
              "y":[640,364,358,531,593,581,787,796],
              "w":[160,107,75,67,58,54,51,46],
              "h":[455,276,221,203,168,151,141,129]},
        "27":{"depth":[150.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[924,1060,918,1130,828,1000,829,799],
              "y":[647,360,353,532,595,586,799,809],
              "w":[148,93,77,66,62,54,55,47],
              "h":[470,287,234,202,174,145,140,135]},
        "28":{"depth":[150.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[924,1059,924,1133,822,1012,1034,790],
              "y":[649,354,351,531,600,586,813,820],
              "w":[164,97,81,72,65,55,56,49],
              "h":[499,295,236,212,186,150,145,141]},
        "29":{"depth":[150.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[921,1063,927,1138,818,1016,1039,792],
              "y":[654,350,342,530,602,589,825,829],
              "w":[172,100,85,73,68,58,57,55],
              "h":[512,306,256,220,193,158,152,145]},
        "30":{"depth":[150.0,250.0,300.0,350.0,400.0,450.0,500.0,550.0],
              "x":[909,1068,926,1146,816,1016,1042,1008],
              "y":[659,342,336,529,607,590,837,844],
              "w":[181,102,89,76,70,61,58,75],
              "h":[533,308,266,223,198,160,157,154]}
    }
}

# -------------------------
# Extract (h, depth) for a zoom
# -------------------------
def get_hz_pairs_from_zoom(zoom_level, unique_by_height=True, reduce="median"):
    """
    Returns (h, z) for the given zoom_level from PERSON_DATA.
    zoom_level: int or str (e.g., 1, "1", 11, "11")
    unique_by_height: collapse duplicate heights to a single value
    reduce: "median" or "mean" when collapsing duplicates
    """
    pdata = PERSON_DATA.get("person_data", {})
    entry = pdata.get(str(zoom_level))
    if not entry:
        raise ValueError(f"No data found for zoom level {zoom_level}")

    h = np.asarray(entry.get("h", []), dtype=float)
    z = np.asarray(entry.get("depth", []), dtype=float)
    if len(h) == 0 or len(z) == 0:
        raise ValueError(f"h/depth arrays are empty for zoom {zoom_level}")

    # sort by height DESC (taller first)
    order = np.argsort(-h)
    h = h[order]; z = z[order]

    if unique_by_height:
        uh, inv = np.unique(h, return_inverse=True)
        if reduce == "median":
            z_by_h = {uh_i: np.median(z[h == uh_i]) for uh_i in uh}
            z = np.array([z_by_h[uh_i] for uh_i in uh])
        else:
            sums = np.zeros_like(uh)
            cnts = np.zeros_like(uh)
            for i, u in enumerate(inv):
                sums[u] += z[i]; cnts[u] += 1
            z = sums / np.maximum(cnts, 1)
        h = uh
        # resort DESC
        order = np.argsort(-h)
        h = h[order]; z = z[order]

    return h, z

# ---------------------------------------------------------
# Interpolators (ranked best -> worst for this use case)
# ---------------------------------------------------------
def build_iso_pchip(heights, depths):
    """#1 Isotonic Regression → PCHIP (smooth + monotone)."""
    from sklearn.isotonic import IsotonicRegression
    from scipy.interpolate import PchipInterpolator

    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)

    # enforce monotone: depth increases as height decreases
    order = np.argsort(-h)
    h = h[order]; z = z[order]
    ir = IsotonicRegression(increasing=True)
    z_iso = ir.fit_transform(-h, z)

    # PCHIP expects increasing x -> reverse both
    f = PchipInterpolator(h[::-1], z_iso[::-1], extrapolate=True)
    return lambda hq: float(f(hq))

def build_pchip(heights, depths):
    """#2 PCHIP (smooth; preserves monotonicity if data already is)."""
    from scipy.interpolate import PchipInterpolator
    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)
    order = np.argsort(h)  # increasing
    f = PchipInterpolator(h[order], z[order], extrapolate=True)
    return lambda hq: float(f(hq))

def build_akima(heights, depths):
    """#3 Akima (smooth; not strictly monotone)."""
    from scipy.interpolate import Akima1DInterpolator
    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)
    order = np.argsort(h)
    f = Akima1DInterpolator(h[order], z[order])
    return lambda hq: float(f(hq))

def build_linear(heights, depths):
    """#4 Linear (fast; piecewise)."""
    from scipy.interpolate import interp1d
    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)
    order = np.argsort(h)
    h = h[order]; z = z[order]
    f = interp1d(h, z, kind="linear", bounds_error=False,
                 fill_value=(z[0], z[-1]))  # clamp ends
    return lambda hq: float(f(hq))

def build_spline(heights, depths, s=0):
    """#5 Univariate Spline (can overshoot; tune 's')."""
    from scipy.interpolate import UnivariateSpline
    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)
    order = np.argsort(h)
    f = UnivariateSpline(h[order], z[order], s=s)  # s=0 exact fit
    return lambda hq: float(f(hq))

def build_isotonic_step(heights, depths):
    """#6 Isotonic (monotone but stepwise)."""
    from sklearn.isotonic import IsotonicRegression
    h = np.asarray(heights, dtype=float)
    z = np.asarray(depths, dtype=float)
    order = np.argsort(-h)
    h = h[order]; z = z[order]
    ir = IsotonicRegression(increasing=True)
    ir.fit(-h, z)
    return lambda hq: float(ir.predict([-float(hq)])[0])

# -------------------------
# printing helpers (for tests)
# -------------------------
def print_single(name, depth_fn, zoom_level, test_heights):
    print(f"\n=== {name} (zoom={zoom_level}) ===")
    for hq in test_heights:
        print(f"h={hq:>4} -> depth={depth_fn(hq):.2f}")

def test_all_methods(zoom_level, test_heights, spline_s=10.0):
    H, Z = get_hz_pairs_from_zoom(zoom_level, unique_by_height=True, reduce="median")

    methods = []
    # build each (wrap in try so missing deps won't crash the loop)
    try:
        methods.append(("Iso+PCHIP", build_iso_pchip(H, Z)))
    except Exception as e:
        print(f"[Skip] Iso+PCHIP: {e}")
    try:
        methods.append(("PCHIP", build_pchip(H, Z)))
    except Exception as e:
        print(f"[Skip] PCHIP: {e}")
    try:
        methods.append(("Akima", build_akima(H, Z)))
    except Exception as e:
        print(f"[Skip] Akima: {e}")
    try:
        methods.append(("Linear", build_linear(H, Z)))
    except Exception as e:
        print(f"[Skip] Linear: {e}")
    try:
        methods.append((f"UnivariateSpline(s={spline_s})", build_spline(H, Z, s=spline_s)))
    except Exception as e:
        print(f"[Skip] Spline: {e}")
    try:
        methods.append(("Isotonic (stepwise)", build_isotonic_step(H, Z)))
    except Exception as e:
        print(f"[Skip] Isotonic: {e}")

    for name, fn in methods:
        print_single(name, fn, zoom_level, test_heights)

# -------------
# MAIN (comment/uncomment)
# -------------
if __name__ == "__main__":
    # Choose zoom level (matches PERSON_DATA["person_data"][str(zoom)])
    zoom_level = 11

    # Provide detection bbox heights to test
    test_heights = [520, 300, 250, 200, 170, 140, 110, 85, 70]

    # ---- Uncomment ONE of the blocks below to test that method ----

    # 1) Iso + PCHIP (best)
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_iso_pchip(H, Z)
    # print_single("Iso+PCHIP", depth_from_h, zoom_level, test_heights)

    # 2) PCHIP
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_pchip(H, Z)
    # print_single("PCHIP", depth_from_h, zoom_level, test_heights)

    # 3) Akima
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_akima(H, Z)
    # print_single("Akima", depth_from_h, zoom_level, test_heights)

    # 4) Linear
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_linear(H, Z)
    # print_single("Linear", depth_from_h, zoom_level, test_heights)

    # 5) Univariate Spline
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_spline(H, Z, s=10.0)
    # print_single("UnivariateSpline(s=10.0)", depth_from_h, zoom_level, test_heights)

    # 6) Isotonic (stepwise)
    # H, Z = get_hz_pairs_from_zoom(zoom_level)
    # depth_from_h = build_isotonic_step(H, Z)
    # print_single("Isotonic (stepwise)", depth_from_h, zoom_level, test_heights)

    # Or compare ALL methods at once (handy sanity check):
    # test_all_methods(zoom_level, test_heights)