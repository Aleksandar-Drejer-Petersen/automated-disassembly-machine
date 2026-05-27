"""
tsp_plot_standalone.py
======================
Self-contained recreation of the TSP path plot used in the bachelor report.
Hand this file to ChatGPT and ask it to tweak colours, layout, fonts, etc.

HOW THE DATA IS STRUCTURED
---------------------------
The plot has N panels, one per segment:
  - "Sweep"         : home → camera positions → detection squares (in visit order)
  - "BIT1"         : grab rack → M4 screw ops (TSP order) → place rack
  - "BIT2"         : grab rack → M5 screw ops + press ops (TSP order) → place rack
  - "Vision Verify" : last rack → camera return to each square with press ops → home

Each waypoint is a tuple:  (x_mm, y_mm, label_string, waypoint_type)

Waypoint types and their visual style:
  "home"   → black star
  "camera" → royalblue square
  "detect" → deepskyblue diamond
  "rack"   → forestgreen triangle
  "screw"  → crimson circle
  "press"   → darkorange circle
  "verify" → mediumpurple plus


HOW THE TSP WORKS IN THIS CODEBASE
------------------------------------
1. SWEEP PHASE
   - Machine visits CAM1 (95, 0) and CAM2 (95, 235) sequentially.
   - At each camera it detects red squares (PCB fixtures) via vision.
   - For each camera, if there are multiple new squares, it TSP-orders them
     starting from the camera position (nearest-neighbour + 2-opt).
   - It drives to each square, centres iteratively, then runs detect_screws()
     which returns a list of {type, machine_dx, machine_dy, has_press, ...}.
   - abs_x = cam_x + machine_dx - CAMERA_OFFSET_X_MM  (-2.137 mm)
   - abs_y = cam_y + machine_dy - CAMERA_OFFSET_Y_MM  (-60.789 mm)
     (camera offset compensates for the camera not being directly above the bit)

2. BIT-GROUP PHASE
   - Operations are grouped by bit: M4→BIT1, M5→BIT2, press→BIT2.
   - Bits are TSP-ordered (starting from last sweep position, between rack coords).
   - Within each bit group, operations are TSP-ordered starting from that rack.
   - Route per bit: GRAB rack → op1 → op2 → ... → opN → PLACE rack

3. VISION VERIFY PHASE
   - Only press operations are verified by camera (screws can't be seen post-op).
   - Press operations are grouped by which detection square they belong to
     (because one camera position covers all ops on the same PCB).
   - Squares with press ops are TSP-ordered from the last bit rack.
   - Route: PLACE last_rack → cam_sq_A → cam_sq_B → HOME

TOTAL DISTANCE = sum of Euclidean mm from each waypoint to the next,
across ALL segments end-to-end (inter-segment transitions included).


KNOWN EXACT COORDINATES (from code constants)
----------------------------------------------
HOME         = (0.0,    0.0)
CAM1         = (95.0,   0.0)
CAM2         = (95.0, 235.038)
BIT1 rack   = (63.537, 361.450)   # M4 screwdriver
BIT2 rack   = (99.537, 361.450)   # M5 screwdriver + press bit
BIT3 rack   = (135.537, 361.450)  # not used in this run
BIT4 rack   = (171.537, 361.450)  # not used in this run

Camera offset applied when computing abs operation position:
  CAMERA_OFFSET_X_MM = -2.137   (subtract from cam_x + machine_dx)
  CAMERA_OFFSET_Y_MM = -60.789  (subtract from cam_y + machine_dy)


OPERATION DATA FROM LAST RUN (16 ops: 6×M4, 6×M5, 4×press)
------------------------------------------------------------
Inferred from main_validation_results.csv fail_details across multiple runs:
  M4  (BIT1): #1, #4, #6, #9, #12, #14
  M5  (BIT2): #2, #5, #7, #10, #13, #15
  press(BIT2): #3, #8, #11, #16

Detection squares (camera centred positions, from TSP plot):
  sq1 camera: approx (75, 45)
  sq2 camera: approx (78, 170)

Operation absolute positions estimated from tsp_path.png.
Replace these with real values by adding this one-liner to main_validation.py
just before the plot_tsp_path() call (around line 616):

    import json, datetime
    with open(f"tsp_data_{datetime.datetime.now().strftime('%H%M%S')}.json","w") as f:
        json.dump(plot_segments, f, indent=2)

That dumps the exact data the plotter receives.
"""

import matplotlib
matplotlib.use("Agg")   # change to "TkAgg" or remove this line if you want a window
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math

# ── Helper ────────────────────────────────────────────────────────────────────
def _d(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# ══════════════════════════════════════════════════════════════════════════════
#  DATA — tweak the coordinates here
#  Format: (x_mm, y_mm, "label", "waypoint_type")
# ══════════════════════════════════════════════════════════════════════════════

# ── Segment 1: Camera sweep ───────────────────────────────────────────────────
# home → CAM1 → sq1 → CAM2 → sq2
sweep = [
    (0.0,    0.0,     "HOME",  "home"),
    (95.0,   0.0,     "CAM1",  "camera"),
    (75.2,   44.8,    "sq1",   "detect"),    # ← estimated; replace with real
    (95.0,   235.038, "CAM2",  "camera"),
    (78.1,   169.5,   "sq2",   "detect"),    # ← estimated; replace with real
]

# ── Segment 2: BIT1 (M4 screws, 6 ops) ──────────────────────────────────────
# These coordinates are abs_x = cam_x + machine_dx - (-2.137)
#                       abs_y = cam_y + machine_dy - (-60.789)
# so sq1 screws cluster around (77, 106) and sq2 screws around (80, 231)
bit1 = [
    (63.537, 361.450, "GRAB\nBIT1", "rack"),
    # M4 screws from sq1 (TSP-ordered by the algorithm)
    (70.3,   97.2,    "#1",          "screw"),
    (84.9,   94.1,    "#4",          "screw"),
    (91.8,   107.3,   "#6",          "screw"),
    # M4 screws from sq2
    (69.8,   222.5,   "#9",          "screw"),
    (83.6,   219.8,   "#12",         "screw"),
    (90.9,   232.4,   "#14",         "screw"),
    (63.537, 361.450, "PLACE\nBIT1","rack"),
]

# ── Segment 3: BIT2 (M5 screws + press, 10 ops) ──────────────────────────────
bit2 = [
    (99.537, 361.450, "GRAB\nBIT2", "rack"),
    # M5 screws sq1
    (63.4,   102.8,   "#2",          "screw"),
    (80.1,   117.5,   "#5",          "screw"),
    (68.2,   121.4,   "#7",          "screw"),
    # press sq1
    (77.1,   108.2,   "#3",          "press"),
    (83.5,   97.8,    "#8",          "press"),
    # M5 screws sq2
    (62.9,   227.9,   "#10",         "screw"),
    (79.4,   242.3,   "#13",         "screw"),
    (67.7,   246.1,   "#15",         "screw"),
    # press sq2
    (76.8,   233.5,   "#11",         "press"),
    (87.2,   222.4,   "#16",         "press"),
    (99.537, 361.450, "PLACE\nBIT2","rack"),
]

# ── Segment 4: Vision verification ───────────────────────────────────────────
# Starts at last rack (BIT2), visits camera position for each square
# that has press ops, then returns home.
verify = [
    (99.537, 361.450, "PLACE\nBIT2", "rack"),
    (75.2,   44.8,    "VER\nsq1",     "verify"),   # same as sq1 camera pos
    (78.1,   169.5,   "VER\nsq2",     "verify"),   # same as sq2 camera pos
    (0.0,    0.0,     "HOME",         "home"),
]

# Assemble all segments
segments = [
    ("Sweep",         sweep),
    ("BIT1",         bit1),
    ("BIT2",         bit2),
    ("Vision Verify", verify),
]

# ══════════════════════════════════════════════════════════════════════════════
#  STYLE — tweak everything visual here
# ══════════════════════════════════════════════════════════════════════════════

STYLE = {
    #  key       color             marker   scatter-size  legend label
    "home":   dict(color="black",        marker="*", size=250, label="Home"),
    "camera": dict(color="royalblue",    marker="s", size=100, label="Camera sweep"),
    "detect": dict(color="deepskyblue",  marker="D", size=70,  label="Detection square"),
    "rack":   dict(color="forestgreen",  marker="^", size=140, label="Bit rack"),
    "screw":  dict(color="crimson",      marker="o", size=85,  label="Screw op"),
    "press":   dict(color="darkorange",   marker="o", size=85,  label="Press op"),
    "verify": dict(color="mediumpurple", marker="P", size=100, label="Vision verify"),
}

ARROW_LW    = 1.1     # line width of path arrows
ARROW_ALPHA = 0.55    # transparency of path arrows
LABEL_FS    = 7       # font size of point labels
TITLE_FS    = 11      # font size of each panel title
LEGEND_FS   = 8       # font size of the shared legend
SUPTITLE_FS = 13      # font size of the overall figure title
PAD_MM      = 18      # extra whitespace (mm) around each panel's content
PANEL_W     = 5.5     # width of each panel in inches
PANEL_H     = 9       # height of the figure in inches
EDGE_C      = "white" # colour of the marker edge rings
EDGE_LW     = 0.5     # width of the marker edge rings
GRID_ALPHA  = 0.2     # transparency of the background grid

# ══════════════════════════════════════════════════════════════════════════════
#  PLOT
# ══════════════════════════════════════════════════════════════════════════════

# Compute total end-to-end distance (across ALL segments, including transitions)
_seq = []
for _, wps in segments:
    _seq.extend(wps)
total_dist = sum(_d((_seq[i][0], _seq[i][1]), (_seq[i+1][0], _seq[i+1][1]))
                for i in range(len(_seq) - 1))

n = len(segments)
fig, axes = plt.subplots(1, n, figsize=(PANEL_W * n, PANEL_H))
if n == 1:
    axes = [axes]

for ax, (title, wps) in zip(axes, segments):
    if len(wps) < 2:
        ax.axis("off")
        ax.set_title(title, fontsize=TITLE_FS, fontweight="bold")
        continue

    xs = [w[0] for w in wps]
    ys = [w[1] for w in wps]
    ax.set_xlim(min(xs) - PAD_MM, max(xs) + PAD_MM)
    ax.set_ylim(min(ys) - PAD_MM, max(ys) + PAD_MM)
    ax.set_aspect("equal", adjustable="box")

    # Arrows along the path
    for i in range(len(wps) - 1):
        x0, y0 = wps[i][0],     wps[i][1]
        x1, y1 = wps[i+1][0],   wps[i+1][1]
        color  = STYLE.get(wps[i+1][3], {}).get("color", "gray")
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle="->", color=color,
                                    lw=ARROW_LW, alpha=ARROW_ALPHA))

    # Points + labels
    for x, y, label, wtype in wps:
        s = STYLE.get(wtype, dict(color="gray", marker="o", size=60))
        ax.scatter(x, y, c=s["color"], marker=s["marker"], s=s["size"],
                   zorder=5, edgecolors=EDGE_C, linewidths=EDGE_LW)
        ax.annotate(label, (x, y),
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=LABEL_FS, color=s["color"],
                    fontweight="bold" if wtype in ("home", "rack") else "normal")

    seg_dist = sum(_d((wps[i][0], wps[i][1]), (wps[i+1][0], wps[i+1][1]))
                   for i in range(len(wps) - 1))
    n_screw = sum(1 for w in wps if w[3] == "screw")
    n_press  = sum(1 for w in wps if w[3] == "press")
    ops_str = f"  ({n_screw}S + {n_press}P)" if (n_screw or n_press) else ""
    ax.set_title(f"{title}{ops_str}\n{seg_dist:.0f} mm", fontsize=TITLE_FS, fontweight="bold")
    ax.set_xlabel("X (mm)", fontsize=9)
    ax.set_ylabel("Y (mm)", fontsize=9)
    ax.grid(True, alpha=GRID_ALPHA)

# Shared legend at the bottom
handles = [mpatches.Patch(color=v["color"], label=v["label"]) for v in STYLE.values()]
fig.legend(handles=handles, loc="lower center", ncol=len(STYLE),
           fontsize=LEGEND_FS, bbox_to_anchor=(0.5, 0.0))

fig.suptitle(f"TSP Path  —  total travel: {total_dist:.0f} mm",
             fontsize=SUPTITLE_FS, fontweight="bold")

plt.tight_layout(rect=[0, 0.06, 1, 0.96])
plt.savefig("tsp_plot_custom.png", dpi=150, bbox_inches="tight")
print(f"Saved: tsp_plot_custom.png   (total travel: {total_dist:.0f} mm)")

# Uncomment to show interactively instead of saving:
# plt.show()
