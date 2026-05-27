"""
main_analyser.py

Drop-in replacement for _execute() in main.py that also captures laser/force
sensor data during each operation and saves an analysis plot to MAIN_ANALYSIS_DIR.

Each plot is named:
    op_<NN>_screw_<subtype>.png   e.g. op_03_screw_M5.png
    op_<NN>_press.png              e.g. op_04_press.png

The plot title shows:
  - Operation number
  - Operation type and subtype
  - SUCCESS / FAILED status
  - Pitch estimated (from detected screw type) and pitch measured/selected
    (from Arduino serial output during unscrewing)
"""

import os
import time

import matplotlib
matplotlib.use("Agg")   # non-interactive backend — no display needed on Pi
import matplotlib.pyplot as plt
import numpy as np

# Standard thread pitch per screw class (mm)
SCREW_TYPE_PITCH_MM = {
    "M1": 0.25,
    "M2": 0.4,
    "M3": 0.5,
    "M4": 0.7,
    "M5": 0.8,
    "M6": 1.0,
    "M8": 1.25,
    "Unknown": None,
}

# ── Unscrew event keywords (mirrors unscrew_analyser.py) ──────────────────────
_UNSCREW_EVENTS = [
    ("UNSCREW: contact detected",          "contact"),
    ("UNSCREW: socket engaged",            "socket engaged"),
    ("UNSCREW: pitch measurement started", "pitch start"),
    ("UNSCREW: measured pitch",            "measured pitch"),
    ("UNSCREW: selected pitch",            "selected pitch"),
    ("UNSCREW: active unscrewing started", "active"),
    ("UNSCREW: screw exit detected",       "exit"),
    ("UNSCREW: done",                      "done"),
    ("UNSCREW ERROR",                      "ERROR"),
]

_UNSCREW_DONE = ["UNSCREW: complete", "UNSCREW ERROR", "control released", "UNSCREW: done"]
_PRESS_DONE    = ["PRESS: complete", "PRESS: failed", "PRESS ERROR"]

PHASE_COLORS_UNSCREW = {
    "contact":        "green",
    "socket engaged": "limegreen",
    "pitch start":    "goldenrod",
    "measured pitch": "darkorange",
    "selected pitch": "peru",
    "active":         "royalblue",
    "exit":           "darkorchid",
    "done":           "seagreen",
    "ERROR":          "red",
}

EVENT_COLORS_PRESS = {
    "contact":     ("tab:green",  "--"),
    "force limit": ("tab:red",    "--"),
    "depth limit": ("tab:orange", "--"),
    "retracted":   ("tab:blue",   ":"),
}

CONTACT_DROP_MM = 2.5

from config import PRESS_SUCCESS_SURFACE_MM


# ── Public entry point ────────────────────────────────────────────────────────
def execute_with_analysis(ser, op, save_dir):
    """Execute a screw/press operation, capture sensor data, and save a plot.

    Replaces main.py's _execute().  Returns (ok: bool, msg: str).
    """
    op_type = op["type"]
    idx     = op["index"]

    # Enable laser stream without the reset_input_buffer() that send_command does
    _raw_write(ser, "LASER STREAM")
    time.sleep(0.2)
    ser.reset_input_buffer()   # discard startup stream noise

    if op_type == "press":
        ok, msg, data = _run_press(ser, timeout=120)
    else:
        ok, msg, data = _run_screw(ser, timeout=120)

    _raw_write(ser, "LASER STOP")
    _raw_write(ser, "FORCE STOP")
    time.sleep(0.15)
    ser.reset_input_buffer()

    try:
        if op_type == "press":
            _plot_press(op, data, ok, save_dir)
        else:
            _plot_screw(op, data, ok, save_dir)
    except Exception as exc:
        print(f"[ANALYSER] Plot failed for op #{idx}: {exc}")

    return ok, msg


# ── Serial helper ─────────────────────────────────────────────────────────────
def _raw_write(ser, cmd):
    """Write to serial without the reset_input_buffer() in send_command."""
    ser.write((cmd.strip() + "\n").encode())
    print(f"[ANALYSER → ARD] {cmd.strip()}")


# ── Screw capture loop ────────────────────────────────────────────────────────
def _run_screw(ser, timeout=120):
    ard_ms      = []
    laser_mm    = []
    events      = []          # (ts_ms, short_label, full_line)
    last_ard_ms = 0

    probe_baseline_mm = None
    measured_pitch    = None
    selected_pitch    = None
    done   = False
    result = None

    _raw_write(ser, "UNSCREW")
    deadline = time.time() + timeout

    while time.time() < deadline and not done:
        if not ser.in_waiting:
            time.sleep(0.001)
            continue
        try:
            line = ser.readline().decode(errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue

        # Laser sample: L:<ms>:<mm>
        if line.startswith("L:"):
            parts = line.split(":")
            if len(parts) == 3:
                try:
                    ms  = int(parts[1])
                    val = float(parts[2])
                    ard_ms.append(ms)
                    laser_mm.append(val)
                    last_ard_ms = ms
                except ValueError:
                    pass
            continue

        print(f"[ARD → PI] {line}")

        if "UNSCREW: started" in line and "probe_baseline=" in line:
            try:
                probe_baseline_mm = float(line.split("probe_baseline=")[1].split()[0])
            except (IndexError, ValueError):
                pass

        for keyword, label in _UNSCREW_EVENTS:
            if keyword.upper() in line.upper():
                if "measured pitch" in keyword:
                    try:
                        val_str = line.split("=")[1].strip().split()[0]
                        measured_pitch = float(val_str)
                        label = f"measured pitch {val_str} mm"
                    except (IndexError, ValueError):
                        pass
                elif "selected pitch" in keyword:
                    try:
                        val_str = line.split("=")[1].strip().split()[0]
                        selected_pitch = float(val_str)
                        label = f"selected pitch {val_str} mm"
                    except (IndexError, ValueError):
                        pass

                ts = last_ard_ms
                try:
                    ts = int(line.split(" at ")[1].split(" ms")[0])
                except (IndexError, ValueError):
                    pass
                events.append((ts, label, line))
                if "done" in keyword.lower() or "ERROR" in keyword.upper():
                    done = True
                break

        for candidate in _UNSCREW_DONE:
            if candidate.upper() in line.upper():
                result = line   # keep full line so caller can inspect the error detail
                done   = True
                break

    if result is None:
        result = "timed out"

    ok = result is not None and ("complete" in result.lower() or "done" in result.lower())

    data = {
        "ard_ms":            ard_ms,
        "laser_mm":          laser_mm,
        "events":            events,
        "probe_baseline_mm": probe_baseline_mm,
        "measured_pitch":    measured_pitch,
        "selected_pitch":    selected_pitch,
    }
    return ok, result, data


# ── Press capture loop ─────────────────────────────────────────────────────────
def _run_press(ser, timeout=120):
    laser_t_ms  = []
    laser_mm    = []
    force_t_ms  = []
    force_N_arr = []
    force_z_mm  = []
    events      = []          # (ts_ms, label)

    probe_baseline_mm  = None
    contact_distance   = None
    contact_t_ms       = None
    contact_z_mm_val   = None
    t0_ms              = None
    result             = None
    done               = False
    force_limit_hit    = False
    depth_limit_hit    = False

    _raw_write(ser, "PRESS ANALYSE")
    deadline = time.time() + timeout

    while time.time() < deadline and not done:
        if not ser.in_waiting:
            time.sleep(0.001)
            continue
        try:
            line = ser.readline().decode(errors="replace").strip()
        except Exception:
            continue
        if not line:
            continue

        # Laser sample: L:<ms>:<mm>
        if line.startswith("L:"):
            parts = line.split(":")
            if len(parts) == 3:
                try:
                    t_ms = int(parts[1])
                    mm   = float(parts[2])
                    if t0_ms is None:
                        t0_ms = t_ms
                    laser_t_ms.append(t_ms - t0_ms)
                    laser_mm.append(mm)
                except ValueError:
                    pass
            continue

        # Force sample: F:<ms>:<N>:<z_mm>
        if line.startswith("F:"):
            parts = line.split(":")
            if len(parts) == 4:
                try:
                    t_ms = int(parts[1])
                    fN   = float(parts[2])
                    zMm  = float(parts[3])
                    if t0_ms is None:
                        t0_ms = t_ms
                    force_t_ms.append(t_ms - t0_ms)
                    force_N_arr.append(fN)
                    force_z_mm.append(zMm)
                except ValueError:
                    pass
            continue

        print(f"[ARD → PI] {line}")

        if "PRESS: started" in line and "probe_baseline=" in line:
            try:
                probe_baseline_mm = float(line.split("probe_baseline=")[1].split()[0])
            except Exception:
                pass

        if "PRESS: contact detected" in line:
            try:
                z_part = line.split("Z=")[1].split()[0].rstrip("mm")
                contact_z_mm_val = float(z_part)
            except Exception:
                pass
            contact_t_ms = laser_t_ms[-1] if laser_t_ms else None
            # find actual laser reading at contact time
            if contact_t_ms is not None and laser_t_ms:
                idx = min(range(len(laser_t_ms)), key=lambda i: abs(laser_t_ms[i] - contact_t_ms))
                contact_distance = laser_mm[idx]
            events.append((contact_t_ms, "contact"))

        if "PRESS: force limit reached" in line:
            force_limit_hit = True
            events.append((laser_t_ms[-1] if laser_t_ms else None, "force limit"))
        if "PRESS: depth limit reached" in line:
            depth_limit_hit = True
            events.append((laser_t_ms[-1] if laser_t_ms else None, "depth limit"))
        if "PRESS: Z retracted" in line:
            events.append((laser_t_ms[-1] if laser_t_ms else None, "retracted"))

        for candidate in _PRESS_DONE:
            if candidate.upper() in line.upper():
                result = line   # keep full line so caller can inspect the error detail
                done   = True
                break

    if result is None:
        result = "timed out"

    # Item displacement metrics (both computed post-contact):
    #
    #   actual_drop  = contact_distance - laser_mm        (laser drop from contact)
    #   z_depth      = force_z - z_at_contact             (Z carriage travel from contact)
    #   item_disp    = z_depth - actual_drop              (item moved in press direction)
    #
    # Case A — item moves DOWN with probe (free vertical press):
    #   laser reads constant → actual_drop ≈ 0, item_disp = z_depth  (positive, grows) ✓
    # Case B — item slides OUT laterally (probe enters empty space):
    #   laser reads rigid surface below → actual_drop = z_depth, item_disp ≈ 0
    #   but actual_drop itself is large and force ≈ 0 → caught by secondary check ✓
    # Case C — item stuck (force safety stop):
    #   actual_drop = z_depth, item_disp ≈ 0, force spikes → force_limit_hit ✓
    #
    # Success = (item_disp >= threshold) OR (actual_drop >= threshold AND no force)
    # Force limit always overrides to FAIL regardless.

    max_item_disp_mm  = float("nan")   # z_depth - actual_drop  (downward displacement)
    max_actual_drop_mm = float("nan")  # actual_drop            (lateral / laser descent)

    if contact_t_ms is not None and contact_distance is not None and laser_t_ms:
        _lt = np.array(laser_t_ms) / 1000.0
        _lm = np.array(laser_mm)
        _ci = int(np.argmin(np.abs(_lt - contact_t_ms / 1000.0)))
        _ad = contact_distance - _lm[_ci:]          # actual_drop, can be negative during retract

        max_actual_drop_mm = float(np.max(np.maximum(0.0, _ad)))

        if len(force_z_mm) > 1:
            _ft = np.array(force_t_ms) / 1000.0
            _fz = np.array(force_z_mm)
            _z0 = float(np.interp(_lt[_ci], _ft, _fz)) if _ft.size else (contact_z_mm_val or 0.0)
            _zd = np.interp(_lt[_ci:], _ft, _fz) - _z0
            _disp = _zd - _ad                        # item displacement (positive = pressed away)
            max_item_disp_mm = float(np.max(np.maximum(0.0, _disp)))

    _item_moved   = (not np.isnan(max_item_disp_mm)   and max_item_disp_mm   >= PRESS_SUCCESS_SURFACE_MM)
    _lateral_move = (not np.isnan(max_actual_drop_mm) and max_actual_drop_mm >= PRESS_SUCCESS_SURFACE_MM
                     and not force_limit_hit)

    if force_limit_hit:
        ok  = False
        msg = "PRESS: force limit hit (safety stop)"
    elif _item_moved:
        ok  = True
        msg = f"PRESS: complete (item displaced {max_item_disp_mm:.1f} mm)"
    elif _lateral_move:
        ok  = True
        msg = f"PRESS: complete (lateral — probe depth {max_actual_drop_mm:.1f} mm, no force)"
    elif result is not None and ("PRESS: complete" in result or "PRESS: failed" in result):
        disp_str = f"{max_item_disp_mm:.1f}" if not np.isnan(max_item_disp_mm) else "?"
        drop_str = f"{max_actual_drop_mm:.1f}" if not np.isnan(max_actual_drop_mm) else "?"
        ok  = False
        msg = (f"PRESS: insufficient displacement "
               f"(item_disp={disp_str} mm, lateral={drop_str} mm, need {PRESS_SUCCESS_SURFACE_MM:.0f} mm)")
    else:
        ok  = False
        msg = result

    data = {
        "laser_t_ms":           laser_t_ms,
        "laser_mm":             laser_mm,
        "force_t_ms":           force_t_ms,
        "force_N":              force_N_arr,
        "force_z_mm":           force_z_mm,
        "events":               events,
        "probe_baseline_mm":    probe_baseline_mm,
        "contact_distance":     contact_distance,
        "contact_t_ms":         contact_t_ms,
        "contact_z_mm_val":     contact_z_mm_val,
        "max_item_disp_mm":     max_item_disp_mm,
        "max_actual_drop_mm":   max_actual_drop_mm,
        "force_limit_hit":      force_limit_hit,
        "depth_limit_hit":      depth_limit_hit,
    }
    return ok, msg, data


# ── Plot: screw ───────────────────────────────────────────────────────────────
def _plot_screw(op, data, ok, save_dir):
    idx     = op["index"]
    subtype = op.get("subtype", "Unknown")
    status  = "SUCCESS" if ok else "FAILED"

    ard_ms            = data["ard_ms"]
    laser_mm_raw      = data["laser_mm"]
    events            = data["events"]
    probe_baseline_mm = data["probe_baseline_mm"]
    measured_pitch    = data["measured_pitch"]
    selected_pitch    = data["selected_pitch"]
    estimated_pitch   = SCREW_TYPE_PITCH_MM.get(subtype)

    if len(laser_mm_raw) < 2:
        _save_no_data_plot(idx, "screw", subtype, status, save_dir)
        return

    t_sec    = np.array([(ms - ard_ms[0]) / 1000.0 for ms in ard_ms])
    laser    = np.array(laser_mm_raw)
    baseline = probe_baseline_mm if probe_baseline_mm is not None else laser[0]
    cum_drop = baseline - laser
    duration_s   = t_sec[-1]
    event_times  = [(ms - ard_ms[0]) / 1000.0 for ms, _, _ in events]
    event_labels = [lbl for _, lbl, _ in events]

    # Title lines
    pitch_parts = []
    if estimated_pitch is not None:
        pitch_parts.append(f"pitch estimated: {estimated_pitch} mm ({subtype})")
    if measured_pitch is not None:
        pitch_parts.append(f"pitch measured: {measured_pitch} mm")
    if selected_pitch is not None:
        pitch_parts.append(f"pitch selected: {selected_pitch} mm")
    subtitle = "  |  ".join(pitch_parts) if pitch_parts else "no pitch data"

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle(
        f"Op #{idx} — SCREW ({subtype}) — {status}"
        f"  |  {len(laser_mm_raw)} samples  |  {duration_s:.1f} s\n"
        f"{subtitle}",
        fontsize=11,
    )

    ax1.plot(t_sec, laser, lw=0.8, color="steelblue")
    if probe_baseline_mm is not None:
        ax1.axhline(probe_baseline_mm, color="orange", lw=1.0, ls="--", alpha=0.7,
                    label=f"probe baseline ({probe_baseline_mm:.2f} mm)")
        ax1.axhline(probe_baseline_mm - CONTACT_DROP_MM, color="green", lw=1.0, ls="--", alpha=0.7,
                    label=f"contact threshold ({probe_baseline_mm - CONTACT_DROP_MM:.2f} mm)")
        ax1.legend(loc="lower left", fontsize=8)
    ax1.set_ylabel("Laser distance (mm)")
    ax1.set_title("Raw laser reading vs time")
    ax1.grid(True, alpha=0.3)

    ax2.fill_between(t_sec, 0, cum_drop, color="tomato", alpha=0.5)
    ax2.axhline(0, color="gray", lw=0.8, ls=":")
    ax2.axhline(CONTACT_DROP_MM, color="orange", lw=1.5, ls="--",
                label=f"contact threshold ({CONTACT_DROP_MM} mm)")
    ax2.set_ylabel("Cumulative change (mm)\n+ = compressed  − = released")
    ax2.set_xlabel("Time (s)")
    ax2.set_title("Cumulative laser change")
    ax2.grid(True, alpha=0.3)

    # Event lines on both axes
    for t, lbl in zip(event_times, event_labels):
        color = _unscrew_event_color(lbl)
        ax1.axvline(t, color=color, lw=1.5, ls="--")
        ax2.axvline(t, color=color, lw=1.5, ls="--")

    # Exit threshold: laser must return within UNSCREW_EXIT_NEAR_BASELINE_MM of probe baseline.
    # Draw as a horizontal line on the raw laser plot so it's immediately readable.
    UNSCREW_EXIT_NEAR_BASELINE_MM = 2.0
    if probe_baseline_mm is not None:
        exit_line = probe_baseline_mm - UNSCREW_EXIT_NEAR_BASELINE_MM
        ax1.axhline(exit_line, color="purple", lw=1.5, ls="--",
                    label=f"exit threshold ({exit_line:.2f} mm = baseline − {UNSCREW_EXIT_NEAR_BASELINE_MM} mm)")
        ax1.legend(loc="lower left", fontsize=8)
    ax2.legend(loc="lower left", fontsize=9)

    plt.tight_layout()

    # Staggered event labels on ax1 (skip pitch — already in title)
    SKIP_LABELS = {"measured pitch", "selected pitch"}
    y_min, y_max = ax1.get_ylim()
    y_range = y_max - y_min
    placed = []
    for t, lbl in zip(event_times, event_labels):
        if any(lbl.startswith(s) for s in SKIP_LABELS):
            continue
        color  = _unscrew_event_color(lbl)
        nearby = sum(1 for pt, _ in placed if abs(pt - t) < 3.0)
        y = y_min + 0.5 + nearby * y_range * 0.18
        placed.append((t, y))
        ax1.text(t + 0.1, y, lbl, color=color, fontsize=7, rotation=90, va="bottom")

    out = os.path.join(save_dir, f"op_{idx:02d}_screw_{subtype}.png")
    plt.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[ANALYSER] Plot saved → {out}")


# ── Plot: press ────────────────────────────────────────────────────────────────
def _plot_press(op, data, ok, save_dir):
    idx    = op["index"]
    status = "SUCCESS" if ok else "FAILED"

    laser_t_ms        = data["laser_t_ms"]
    laser_mm_raw      = data["laser_mm"]
    force_t_ms        = data["force_t_ms"]
    force_N_raw       = data["force_N"]
    force_z_mm_raw    = data["force_z_mm"]
    events            = data["events"]
    probe_baseline_mm = data["probe_baseline_mm"]
    contact_distance  = data["contact_distance"]
    contact_t_ms      = data["contact_t_ms"]
    contact_z_mm_val  = data["contact_z_mm_val"]

    if not laser_mm_raw:
        _save_no_data_plot(idx, "press", "press", status, save_dir)
        return

    laser_t  = np.array(laser_t_ms)  / 1000.0
    laser_mm = np.array(laser_mm_raw)
    force_t  = np.array(force_t_ms)  / 1000.0 if force_t_ms  else np.array([])
    force_N  = np.array(force_N_raw)           if force_N_raw else np.array([])
    force_z  = np.array(force_z_mm_raw)        if force_z_mm_raw else np.array([])
    duration_s = laser_t[-1] if len(laser_t) else 0.0

    if probe_baseline_mm is None and laser_mm.size:
        probe_baseline_mm = laser_mm[0]

    contact_idx = None
    if contact_t_ms is not None:
        contact_t_s = contact_t_ms / 1000.0
        diffs = np.abs(laser_t - contact_t_s)
        contact_idx = int(np.argmin(diffs))
        if contact_distance is None:
            contact_distance = laser_mm[contact_idx]

    # Post-contact curves for bottom panel.
    # Both lines start at 0 at the moment of contact.
    #   z_depth_arr  = how far the Z carriage descended from the contact point
    #   actual_drop  = how far the laser reading dropped from the contact distance
    #
    # Reading the gap:
    #   Lines diverge  → item moving WITH the probe (probe in contact, item displaced)
    #   Lines converge → item fell away, probe now descending through empty space
    #   Gap = item displacement; either line crossing the threshold = success
    post_t          = np.array([])
    actual_drop_arr = np.array([])
    z_depth_arr     = np.array([])
    if contact_idx is not None and contact_distance is not None:
        post_t          = laser_t[contact_idx:]
        actual_drop_arr = np.maximum(0.0, contact_distance - laser_mm[contact_idx:])
        if force_t.size and force_z.size:
            z_at_c      = float(np.interp(laser_t[contact_idx], force_t, force_z))
            z_depth_arr = np.maximum(0.0, np.interp(post_t, force_t, force_z) - z_at_c)

    fig, axes_plot = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(
        f"Op #{idx} — PRESS — {status}"
        f"  |  {len(laser_mm)} laser samples  |  {len(force_N)} force samples  |  {duration_s:.1f} s",
        fontsize=11,
    )

    ax1 = axes_plot[0]
    ax1.plot(laser_t, laser_mm, color="steelblue", lw=1.2, label="Laser distance")
    if probe_baseline_mm is not None:
        ax1.axhline(probe_baseline_mm, color="grey", lw=0.8, ls=":",
                    label=f"Baseline {probe_baseline_mm:.2f} mm")
    if contact_idx is not None and contact_distance is not None:
        ax1.axhline(contact_distance, color="tab:green", lw=0.8, ls="--",
                    label=f"Contact {contact_distance:.2f} mm")
    ax1.set_ylabel("Distance (mm)")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2 = axes_plot[1]
    if force_t.size:
        ax2.plot(force_t, force_N, color="tab:red", lw=1.2, label="Force")
        ax2.axhline(5.0, color="darkred", lw=0.8, ls="--", label="Max force 5.0 N")
    else:
        ax2.text(0.5, 0.5, "No force data", ha="center", va="center",
                 transform=ax2.transAxes, fontsize=11, color="grey")
    ax2.set_ylabel("Force (N)")
    ax2.legend(fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    ax3 = axes_plot[2]
    if post_t.size:
        ax3.plot(post_t, actual_drop_arr, color="steelblue", lw=1.5,
                 label="Laser depth from contact  (= 0 while item moves with probe)")
        if z_depth_arr.size:
            ax3.plot(post_t, z_depth_arr, color="darkorange", lw=1.5,
                     label="Z carriage depth from contact  (always rises while pressing)")
            # Shade the gap — this is the item displacement
            ax3.fill_between(post_t, actual_drop_arr, z_depth_arr,
                             where=(z_depth_arr >= actual_drop_arr),
                             alpha=0.25, color="darkorange",
                             label="Gap = item displacement (diverge→contact, converge→item fell)")
        ax3.axhline(PRESS_SUCCESS_SURFACE_MM, color="seagreen", lw=1.2, ls="--",
                    label=f"Success threshold ({PRESS_SUCCESS_SURFACE_MM:.0f} mm)")
        ax3.axhline(0, color="grey", lw=0.6, ls=":")
    else:
        ax3.text(0.5, 0.5, "No contact data", ha="center", va="center",
                 transform=ax3.transAxes, fontsize=11, color="grey")
    ax3.set_ylabel("Depth from contact point (mm)")
    ax3.set_xlabel("Time (s)")
    ax3.legend(fontsize=8, loc="upper right")
    ax3.grid(True, alpha=0.3)

    # Event markers on all subplots
    for t_ms_ev, label in events:
        if t_ms_ev is None:
            continue
        t_s = t_ms_ev / 1000.0
        color, ls = EVENT_COLORS_PRESS.get(label, ("black", "--"))
        for ax in axes_plot:
            ax.axvline(t_s, color=color, lw=1.0, ls=ls, alpha=0.7)
        axes_plot[0].text(t_s, axes_plot[0].get_ylim()[0], f" {label}",
                          color=color, fontsize=7, va="bottom", rotation=90)

    plt.tight_layout()
    out = os.path.join(save_dir, f"op_{idx:02d}_press.png")
    plt.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[ANALYSER] Plot saved → {out}")


# ── Helpers ───────────────────────────────────────────────────────────────────
def _unscrew_event_color(lbl):
    for key, color in PHASE_COLORS_UNSCREW.items():
        if lbl.startswith(key):
            return color
    return "black"


def _save_no_data_plot(idx, op_type, subtype, status, save_dir):
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.text(0.5, 0.5,
            f"Op #{idx} — {op_type.upper()} ({subtype}) — {status}\nNo sensor data captured.",
            ha="center", va="center", fontsize=14, transform=ax.transAxes)
    ax.axis("off")
    suffix = f"screw_{subtype}" if op_type == "screw" else "press"
    out = os.path.join(save_dir, f"op_{idx:02d}_{suffix}.png")
    plt.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[ANALYSER] No-data plot saved → {out}")
