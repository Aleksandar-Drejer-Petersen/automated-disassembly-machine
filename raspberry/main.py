"""
Automated disassembly — main script.

Workflow:
  For each camera position (CAM1, CAM2, ...):
    1. Move to camera position
    2. Capture image and find all red squares
    3. Skip squares already visited from a previous camera
    4. For each new square: move to it → iterative centering → detect operations
  Once all cameras done:
    5. Group operations by bit; TSP-order within each group
    6. Execute: grab bit → move → UNSCREW/PRESS → home Z → place bit
    7. Home and report failures

Camera layout:
  CAM1 and CAM2 are the default positions (same X, different Y).
  Extra cameras follow at increasing X offsets with the same Y pair,
  forming a grid: CAM1/2, CAM3/4, CAM5/6, …

Usage:
    python main.py
    python main.py --no-execute
"""

import argparse
import os
import glob
import threading
import time

from config import (
    MM_PER_PIXEL_TXT_PATH,
    CAMERA_OFFSET_X_MM, CAMERA_OFFSET_Y_MM,
    VISION_CURRENT_RUN_DIR,
    MAIN_ANALYSIS_DIR,
)

os.makedirs(VISION_CURRENT_RUN_DIR, exist_ok=True)
os.makedirs(MAIN_ANALYSIS_DIR, exist_ok=True)


def _clear_current_run_images():
    for f in glob.glob(os.path.join(VISION_CURRENT_RUN_DIR, "*")):
        try:
            os.remove(f)
        except OSError:
            pass


def _clear_main_analysis():
    for f in glob.glob(os.path.join(MAIN_ANALYSIS_DIR, "*")):
        try:
            os.remove(f)
        except OSError:
            pass

from camera import capture_image, load_mm_per_pixel
from vision import find_red_square_offset, find_all_red_squares, detect_screws, check_press_at_pixel
from serial_comm import open_serial, send_command, wait_for_message, wait_for_any_message
from main_analyser import execute_with_analysis

# ── Camera positions ──────────────────────────────────────────────────────────
# Must match Arduino cameraPositions[].  Add more cameras here as pairs at the
# same Y values but offset in X — the code handles any number automatically.
CAMERA_SWEEP = [
    ("CAM1", 95.0, 0.0),
    ("CAM2", 95.0, 235.038),
]

# Machine travel limits — used to discard physically impossible detections
X_MIN_MM, X_MAX_MM = 5.0, 535.0
Y_MIN_MM, Y_MAX_MM = 5.0, 366.0

# ── Bit mapping ──────────────────────────────────────────────────────────────
BIT_FOR_SCREW_TYPE = {
    "M4":      "BIT1",
    "M5":      "BIT2",
    "M6":      "BIT3",
    "M8":      "BIT4",
    "Unknown": "BIT2",
}
PRESS_BIT = "BIT2"

CENTER_TOLERANCE_MM  = 0.5
MAX_CENTER_ITERS     = 4
ALREADY_VISITED_DIST = 20.0   # mm — squares closer than this are the same square


# ── TSP (multi-start nearest-neighbour + 2-opt, open path) ───────────────────
def _d(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _nn_from(coords, first):
    n = len(coords)
    unvisited = list(range(n))
    path = [first]
    unvisited.remove(first)
    while unvisited:
        last = path[-1]
        nxt = min(unvisited, key=lambda j: _d(coords[last], coords[j]))
        path.append(nxt)
        unvisited.remove(nxt)
    return path


def _2opt(coords, path, start_xy=None, end_xy=None):
    n = len(path)
    if n < 4:
        return list(path)
    path = list(path)
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 2, n):
                left  = start_xy if (i == 0 and start_xy is not None) else coords[path[i]]
                right = (end_xy if (j == n - 1 and end_xy is not None)
                         else (coords[path[j + 1]] if j + 1 < n else None))
                d_old = _d(left, coords[path[i + 1]])
                d_new = _d(left, coords[path[j]])
                if right is not None:
                    d_old += _d(coords[path[j]],     right)
                    d_new += _d(coords[path[i + 1]], right)
                if d_new < d_old - 1e-10:
                    path[i + 1:j + 1] = path[i + 1:j + 1][::-1]
                    improved = True
    return path


def _tour_cost(coords, path, start_xy=None, end_xy=None):
    cost = sum(_d(coords[path[i]], coords[path[i + 1]]) for i in range(len(path) - 1))
    if start_xy is not None:
        cost += _d(start_xy, coords[path[0]])
    if end_xy is not None:
        cost += _d(coords[path[-1]], end_xy)
    return cost


def tsp(coords, start_xy=None, end_xy=None):
    n = len(coords)
    if n == 0:
        return []
    if n == 1:
        return [0]
    best_path, best_cost = None, float("inf")
    for first in range(n):
        path = _2opt(coords, _nn_from(coords, first), start_xy, end_xy)
        cost = _tour_cost(coords, path, start_xy, end_xy)
        if cost < best_cost:
            best_cost, best_path = cost, list(path)
    return best_path


# ── Helpers ───────────────────────────────────────────────────────────────────
def _already_visited(sq, visited):
    return any(_d(sq, v) < ALREADY_VISITED_DIST for v in visited)


def _execute(ser, op_type):
    import time
    if op_type == "press":
        # Press success is determined by execute_with_analysis (surface movement).
        # This bare fallback has no sensor data, so any non-error completion is
        # treated as indeterminate — callers should use execute_with_analysis instead.
        send_command(ser, "PRESS")
        result = wait_for_any_message(ser, ["PRESS: complete", "PRESS: failed", "PRESS ERROR"], timeout=120)
        if result is None:
            return False, "timed out"
        time.sleep(0.15)
        ser.reset_input_buffer()
        if "ERROR" in result or result == "PRESS: failed":
            return False, result
        # Cannot determine surface movement without sensor data; treat as unknown.
        return False, "PRESS: no sensor data (use execute_with_analysis)"
    else:
        send_command(ser, "UNSCREW")
        result = wait_for_any_message(
            ser, ["UNSCREW: complete", "UNSCREW ERROR", "control released"], timeout=120
        )
        if result is None:
            return False, "timed out"
        time.sleep(0.15)
        ser.reset_input_buffer()
        return "complete" in result.lower(), result


def _centre_on_square(ser, approx_x, approx_y, mm_per_pixel_x, mm_per_pixel_y, label):
    """Iteratively centre the camera on the red square. Returns (final_x, final_y)."""
    machine_x, machine_y = approx_x, approx_y
    for iteration in range(MAX_CENTER_ITERS):
        img_path = os.path.join(VISION_CURRENT_RUN_DIR, f"centre_{label}_iter{iteration}.jpg")
        capture_image(img_path)

        dx, dy = find_red_square_offset(img_path, mm_per_pixel_x, mm_per_pixel_y)
        print(f"  [{label}] iter {iteration + 1}: dx={dx:+.3f} mm, dy={dy:+.3f} mm")

        if abs(dx) <= CENTER_TOLERANCE_MM and abs(dy) <= CENTER_TOLERANCE_MM:
            print(f"  [{label}] Centred.")
            break

        send_command(ser, f"x {dx:.3f}")
        wait_for_message(ser, "SYSTEM IS AT X", timeout=60)
        send_command(ser, f"y {dy:.3f}")
        wait_for_message(ser, "SYSTEM IS AT Y", timeout=60)
        time.sleep(0.15)   # let the machine settle before the next capture
        machine_x += dx
        machine_y += dy
    else:
        print(f"  [{label}] WARNING: not fully centred after {MAX_CENTER_ITERS} iterations.")
    return machine_x, machine_y


# ── TSP Path Visualisation ────────────────────────────────────────────────────
def plot_tsp_path(segments, save_path, total_dist=None):
    if total_dist is None:
        total_dist = sum(
            _d((wps[i][0], wps[i][1]), (wps[i + 1][0], wps[i + 1][1]))
            for _, wps in segments
            for i in range(len(wps) - 1)
        )
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("  WARNING: matplotlib not installed — TSP path plot skipped.")
        print(f"  Total planned travel: {total_dist:.0f} mm")
        return total_dist

    STYLE = {
        "home":   dict(color="black",        marker="*", size=250, label="Home"),
        "camera": dict(color="royalblue",    marker="s", size=100, label="Camera sweep"),
        "detect": dict(color="deepskyblue",  marker="D", size=70,  label="Detection square"),
        "rack":   dict(color="forestgreen",  marker="^", size=140, label="Bit rack"),
        "screw":  dict(color="crimson",      marker="o", size=85,  label="Screw op"),
        "press":   dict(color="darkorange",   marker="o", size=85,  label="Press op"),
        "verify": dict(color="mediumpurple", marker="P", size=100, label="Vision verify"),
    }

    PAD = 18
    n   = len(segments)
    fig, axes = plt.subplots(1, n, figsize=(5.5 * n, 9))
    if n == 1:
        axes = [axes]

    for ax, (title, wps) in zip(axes, segments):
        if len(wps) < 2:
            ax.axis("off")
            ax.set_title(title, fontsize=11, fontweight="bold")
            continue

        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        ax.set_xlim(min(xs) - PAD, max(xs) + PAD)
        ax.set_ylim(min(ys) - PAD, max(ys) + PAD)
        ax.set_aspect("equal", adjustable="datalim")

        for i in range(len(wps) - 1):
            x0, y0 = wps[i][0],     wps[i][1]
            x1, y1 = wps[i + 1][0], wps[i + 1][1]
            color = STYLE.get(wps[i + 1][3], {}).get("color", "gray")
            ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                        arrowprops=dict(arrowstyle="->", color=color,
                                        lw=1.1, alpha=0.55))

        for x, y, label, wtype in wps:
            s = STYLE.get(wtype, dict(color="gray", marker="o", size=60))
            ax.scatter(x, y, c=s["color"], marker=s["marker"], s=s["size"],
                       zorder=5, edgecolors="white", linewidths=0.5)
            ax.annotate(label, (x, y),
                        textcoords="offset points", xytext=(6, 4),
                        fontsize=7, color=s["color"],
                        fontweight="bold" if wtype in ("home", "rack") else "normal")

        seg_dist = sum(
            _d((wps[i][0], wps[i][1]), (wps[i + 1][0], wps[i + 1][1]))
            for i in range(len(wps) - 1)
        )
        n_screw = sum(1 for w in wps if w[3] == "screw")
        n_press  = sum(1 for w in wps if w[3] == "press")
        ops_str = f"  ({n_screw}S + {n_press}P)" if (n_screw or n_press) else ""
        ax.set_title(f"{title}{ops_str}\n{seg_dist:.0f} mm",
                     fontsize=11, fontweight="bold")
        ax.set_xlabel("X (mm)", fontsize=9)
        ax.set_ylabel("Y (mm)", fontsize=9)
        ax.grid(True, alpha=0.2)

    handles = [mpatches.Patch(color=v["color"], label=v["label"])
               for v in STYLE.values()]
    fig.legend(handles=handles, loc="lower center", ncol=len(STYLE),
               fontsize=8, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle(f"TSP Path  —  total travel: {total_dist:.0f} mm",
                 fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0.06, 1, 0.96])
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  TSP path plot saved → {save_path}")
    print(f"  Total planned travel distance: {total_dist:.0f} mm")
    return total_dist


def _laser_recovery(ser):
    """Stream live laser readings until operator presses Enter (retry) or types 'skip'."""
    def _write(cmd):
        ser.write((cmd + "\n").encode())
        print(f"[PI → ARD] {cmd}")

    print()
    print("  ┌─ LASER RECOVERY ──────────────────────────────────────────────┐")
    print("  │  Laser reading is out of range — check for an obstruction.   │")
    print("  │  Live reading shown below. Press ENTER to retry this op,     │")
    print("  │  or type  skip  + ENTER to cancel it and move on.            │")
    print("  └───────────────────────────────────────────────────────────────┘")

    _write("LASER STREAM")
    time.sleep(0.1)
    ser.reset_input_buffer()

    stop = threading.Event()

    def _stream():
        while not stop.is_set():
            if ser.in_waiting:
                try:
                    line = ser.readline().decode(errors="replace").strip()
                    if line.startswith("L:"):
                        parts = line.split(":")
                        if len(parts) == 3:
                            print(f"\r  Laser: {float(parts[2]):6.2f} mm    ",
                                  end="", flush=True)
                except Exception:
                    pass
            else:
                time.sleep(0.01)

    t = threading.Thread(target=_stream, daemon=True)
    t.start()

    try:
        user_input = input().strip().lower()
    finally:
        stop.set()
        t.join(timeout=1.0)
        print()
        _write("LASER STOP")
        time.sleep(0.15)
        ser.reset_input_buffer()

    return user_input != "skip"


def _execute_with_retry(ser, op, save_dir):
    """Run execute_with_analysis; offer one laser-recovery retry on baseline errors."""
    ok, msg = execute_with_analysis(ser, op, save_dir)
    if ok or "baseline" not in msg.lower():
        return ok, msg
    if _laser_recovery(ser):
        print(f"  Retrying #{op['index']}...")
        ok, msg = execute_with_analysis(ser, op, save_dir)
    return ok, msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-execute", action="store_true",
                        help="Detect and plan operations but skip execution")
    args = parser.parse_args()

    mm_per_pixel_x, mm_per_pixel_y = load_mm_per_pixel(MM_PER_PIXEL_TXT_PATH)
    ser = open_serial()

    try:
        # ── Clear previous run images and analysis plots ──────────────────────
        _clear_current_run_images()
        _clear_main_analysis()
        print("vision_current_run and main_analysis_photos cleared.")

        # ── Home ─────────────────────────────────────────────────────────────
        print("\n=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        # ── Camera sweep: detect and map all operations ───────────────────────
        all_operations  = []
        visited_squares = []
        sweep_waypoints = [(0.0, 0.0, "HOME", "home")]
        op_index  = 1
        sq_global = 0

        for cam_name, cam_x, cam_y in CAMERA_SWEEP:
            print(f"\n{'='*55}")
            print(f"=== {cam_name} (X={cam_x}, Y={cam_y}) ===")
            print(f"{'='*55}")

            send_command(ser, cam_name.lower())
            wait_for_message(ser, f"SYSTEM IS AT {cam_name}", timeout=60)
            sweep_waypoints.append((cam_x, cam_y, cam_name, "camera"))

            # Take picture and find all red squares
            sweep_img = os.path.join(VISION_CURRENT_RUN_DIR, f"sweep_{cam_name.lower()}.jpg")
            capture_image(sweep_img)

            found = find_all_red_squares(sweep_img, mm_per_pixel_x, mm_per_pixel_y)
            abs_squares = [(cam_x + dx, cam_y + dy) for dx, dy in found]

            # Discard positions outside reachable machine travel
            valid_squares = [
                sq for sq in abs_squares
                if X_MIN_MM <= sq[0] <= X_MAX_MM and Y_MIN_MM <= sq[1] <= Y_MAX_MM
            ]
            discarded = len(abs_squares) - len(valid_squares)
            if discarded:
                print(f"  Discarded {discarded} out-of-bounds detection(s).")

            print(f"  {len(valid_squares)} valid red square(s) found.")

            # Split into new vs already visited
            new_squares = [sq for sq in valid_squares if not _already_visited(sq, visited_squares)]
            skip_count  = len(valid_squares) - len(new_squares)
            if skip_count:
                print(f"  Skipping {skip_count} already-visited square(s).")
            if not new_squares:
                print(f"  Nothing new to visit at {cam_name}.")
                continue

            print(f"  Visiting {len(new_squares)} new square(s) in detection order.")

            for rank, (approx_x, approx_y) in enumerate(new_squares):
                sq_global += 1
                label = f"sq{sq_global}"
                print(f"\n--- Square {sq_global} ({cam_name} #{rank + 1}): "
                      f"approx X={approx_x:.2f}, Y={approx_y:.2f} ---")

                send_command(ser, f"to {approx_x:.3f} {approx_y:.3f} 0.000")
                wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

                rs_x, rs_y = _centre_on_square(
                    ser, approx_x, approx_y,
                    mm_per_pixel_x, mm_per_pixel_y, label
                )
                print(f"  Centred at X={rs_x:.3f}, Y={rs_y:.3f}")
                sweep_waypoints.append((rs_x, rs_y, label, "detect"))

                # Capture final image and detect operations
                detect_img = os.path.join(VISION_CURRENT_RUN_DIR, f"detect_{label}.jpg")
                debug_path = os.path.join(VISION_CURRENT_RUN_DIR, f"debug_{label}.jpg")
                capture_image(detect_img)

                screw_results, _ = detect_screws(
                    detect_img, debug_path, mm_per_pixel_x, mm_per_pixel_y
                )

                visited_squares.append((rs_x, rs_y))

                if not screw_results:
                    print(f"  No operations detected on {label}.")
                    continue

                for s in screw_results:
                    abs_x = rs_x + s["machine_dx"] - CAMERA_OFFSET_X_MM
                    abs_y = rs_y + s["machine_dy"] - CAMERA_OFFSET_Y_MM
                    op_type = "press" if s["has_press"] else "screw"
                    bit    = (PRESS_BIT if op_type == "press"
                               else BIT_FOR_SCREW_TYPE.get(s["screw_type"], "BIT2"))
                    all_operations.append({
                        "index":   op_index,
                        "square":  sq_global,
                        "type":    op_type,
                        "bit":    bit,
                        "abs_x":   abs_x,
                        "abs_y":   abs_y,
                        "subtype": s["screw_type"] if op_type == "screw" else "press",
                        # Detection image context — used for post-execution camera verification
                        "cam_x":   rs_x,        # camera position (centred on square)
                        "cam_y":   rs_y,
                        "px":      s["cx"],     # pixel location in the detection image
                        "py":      s["cy"],
                        "radius":  s["radius"], # pixel radius at detection
                    })
                    op_index += 1

        # ── Operation plan ────────────────────────────────────────────────────
        print(f"\n=== OPERATION PLAN ({len(all_operations)} total) ===")
        for op in all_operations:
            print(f"  #{op['index']:2d}  sq{op['square']}  {op['type']:5s}"
                  f" ({op['subtype']:7s})  bit={op['bit']}"
                  f"  X={op['abs_x']:.2f}  Y={op['abs_y']:.2f}")

        if not all_operations:
            print("No operations detected. Exiting.")
            return

        # ── TSP ordering and path plot ─────────────────────────────────────────
        BIT_RACK_XY = {
            "BIT1": (63.537,  361.450),
            "BIT2": (99.537,  361.450),
            "BIT3": (135.537, 361.450),
            "BIT4": (171.537, 361.450),
        }

        bit_order = []
        seen = set()
        for op in all_operations:
            if op["bit"] not in seen:
                bit_order.append(op["bit"])
                seen.add(op["bit"])
        grouped = {t: [op for op in all_operations if op["bit"] == t] for t in bit_order}

        ordered_groups = {}
        for bit_name in bit_order:
            ops_g     = grouped[bit_name]
            op_coords = [(op["abs_x"], op["abs_y"]) for op in ops_g]
            rack_xy   = BIT_RACK_XY.get(bit_name, (0.0, 0.0))
            ordered_groups[bit_name] = [
                ops_g[i] for i in tsp(op_coords, start_xy=rack_xy, end_xy=rack_xy)
            ]

        bit_segments = []
        for bit_name in bit_order:
            rack_x, rack_y = BIT_RACK_XY.get(bit_name, (0.0, 0.0))
            bit_wps = [(rack_x, rack_y, f"GRAB\n{bit_name}", "rack")]
            for op in ordered_groups[bit_name]:
                wtype = "press" if op["type"] == "press" else "screw"
                bit_wps.append((op["abs_x"], op["abs_y"], f"#{op['index']}", wtype))
            bit_wps.append((rack_x, rack_y, f"PLACE\n{bit_name}", "rack"))
            bit_segments.append((bit_name, bit_wps))

        plot_segments = [("Sweep", sweep_waypoints)] + bit_segments
        _seq = list(sweep_waypoints)
        for _, bit_wps in bit_segments:
            _seq.extend(bit_wps)
        total_dist = sum(
            _d((_seq[i][0], _seq[i][1]), (_seq[i + 1][0], _seq[i + 1][1]))
            for i in range(len(_seq) - 1)
        )
        tsp_plot_path = os.path.join(VISION_CURRENT_RUN_DIR, "tsp_path.png")
        print(f"\n=== TSP PATH ===")
        plot_tsp_path(plot_segments, tsp_plot_path, total_dist)

        if args.no_execute:
            print("\n--no-execute set. Done.")
            return

        # ── Execute ───────────────────────────────────────────────────────────
        failures = []
        last_rack_xy = (0.0, 0.0)

        for bit_name in bit_order:
            ops = ordered_groups[bit_name]

            print(f"\n=== Grabbing {bit_name} ({len(ops)} operation(s)) ===")
            send_command(ser, f"grab {bit_name.lower()}")
            grab_result = wait_for_any_message(
                ser, ["GRAB: complete.", "GRAB ERROR"], timeout=90
            )
            if grab_result is None or "ERROR" in grab_result:
                print(f"  GRAB FAILED for {bit_name} — skipping {len(ops)} operation(s).")
                for op in ops:
                    failures.append((op, f"bit grab failed ({bit_name})"))
                continue

            for op in ops:
                print(f"\n  --- #{op['index']} sq{op['square']} {op['type']}"
                      f" ({op['subtype']}) X={op['abs_x']:.2f}, Y={op['abs_y']:.2f} ---")

                send_command(ser, f"to {op['abs_x']:.3f} {op['abs_y']:.3f} 0.000")
                wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

                ok, msg = _execute_with_retry(ser, op, MAIN_ANALYSIS_DIR)

                if op["type"] == "press":
                    # Press result is deferred — camera verifies after bit placement.
                    # Still home Z on sensor failure so the next XY move is safe.
                    if not ok:
                        time.sleep(0.3)
                        ser.reset_input_buffer()
                        send_command(ser, "h z")
                        wait_for_message(ser, "Axis Z safe. Position = 0.", timeout=30)
                else:
                    if ok:
                        print(f"  ✓ #{op['index']} succeeded")
                    else:
                        print(f"  ✗ #{op['index']} FAILED: {msg}")
                        failures.append((op, msg))
                        print("  → Homing Z after failure...")
                        send_command(ser, "h z")
                        wait_for_message(ser, "Axis Z safe. Position = 0.", timeout=30)

            print(f"\n=== Placing {bit_name} ===")
            send_command(ser, f"place {bit_name.lower()}")
            place_result = wait_for_any_message(
                ser, ["PLACE: complete.", "PLACE ERROR"], timeout=90
            )
            if place_result is None or "ERROR" in place_result:
                print(f"  WARNING: place failed for {bit_name}.")
            last_rack_xy = BIT_RACK_XY.get(bit_name, last_rack_xy)

        # ── Camera verification pass for press operations ─────────────────────
        # One photo per square (same camera position used during detection).
        # All press ops on the same square are checked in that single photo
        # at the exact pixel locations where they were originally found.
        press_ops = [op for op in all_operations if op["type"] == "press"]
        vision_results = {}   # op index → (vision_ok, ratio)

        if press_ops:
            print(f"\n=== Camera verification for {len(press_ops)} press operation(s) ===")

            # Group by square, preserve insertion order
            seen_sq = []
            press_by_square = {}
            for op in press_ops:
                sq = op["square"]
                if sq not in press_by_square:
                    press_by_square[sq] = []
                    seen_sq.append(sq)
                press_by_square[sq].append(op)

            # TSP over the square camera positions
            sq_cam_coords = [(press_by_square[sq][0]["cam_x"],
                              press_by_square[sq][0]["cam_y"]) for sq in seen_sq]
            sq_order = [seen_sq[i] for i in tsp(sq_cam_coords, start_xy=last_rack_xy)]

            for sq_id in sq_order:
                sq_ops  = press_by_square[sq_id]
                cam_x   = sq_ops[0]["cam_x"]
                cam_y   = sq_ops[0]["cam_y"]
                print(f"\n  Square {sq_id}: moving to camera position "
                      f"X={cam_x:.3f} Y={cam_y:.3f}")

                ser.reset_input_buffer()
                send_command(ser, f"to {cam_x:.3f} {cam_y:.3f} 0.000")
                wait_for_message(ser, "SYSTEM IS AT POSITION", timeout=60)

                verify_img = os.path.join(MAIN_ANALYSIS_DIR, f"sq{sq_id}_verify.jpg")
                capture_image(verify_img)

                for op in sq_ops:
                    verify_debug = os.path.join(MAIN_ANALYSIS_DIR,
                                                f"press{op['index']:02d}_verify_debug.jpg")
                    try:
                        v_ok, v_ratio, _ = check_press_at_pixel(
                            verify_img, op["px"], op["py"], op["radius"], verify_debug
                        )
                    except Exception as exc:
                        print(f"  WARNING: vision check failed for press #{op['index']}: {exc}")
                        v_ok, v_ratio = None, None
                    vision_results[op["index"]] = (v_ok, v_ratio)

        # ── Home and report ───────────────────────────────────────────────────
        print("\n=== Homing machine ===")
        send_command(ser, "h")
        wait_for_message(ser, "SYSTEM IS HOMED", timeout=120)

        screw_ops  = [op for op in all_operations if op["type"] != "press"]
        screw_fail = len(failures)   # failures list only contains screw failures now
        screw_ok   = len(screw_ops) - screw_fail

        print(f"\n{'=' * 55}")
        print(f"  SCREW RESULTS: {screw_ok}/{len(screw_ops)} succeeded")
        if failures:
            for op, reason in failures:
                print(f"    ✗ sq{op['square']} {op['subtype']} "
                      f"X={op['abs_x']:.3f} Y={op['abs_y']:.3f}  —  {reason}")
        else:
            if screw_ops:
                print("  All screw operations completed successfully.")

        if vision_results:
            press_pass = sum(1 for v_ok, _ in vision_results.values() if v_ok)
            print(f"\n  PRESS RESULTS (camera): {press_pass}/{len(press_ops)} succeeded")
            for op in press_ops:
                v_ok, v_ratio = vision_results.get(op["index"], (None, None))
                tag   = ("PASS" if v_ok else "FAIL" if v_ok is not None else "ERROR")
                r_str = f"  yel={v_ratio:.3f}" if v_ratio is not None else ""
                print(f"    {'✓' if v_ok else '✗'} press #{op['index']} sq{op['square']} "
                      f"X={op['abs_x']:.2f} Y={op['abs_y']:.2f}: {tag}{r_str}")
        elif press_ops:
            print(f"\n  PRESS RESULTS: camera verification skipped")
        print(f"{'=' * 55}")

    finally:
        ser.close()
        print("\nSerial closed.")


if __name__ == "__main__":
    main()
