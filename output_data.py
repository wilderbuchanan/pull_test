#!/usr/bin/env python3
# Produces a clean 1-page PDF from data/<test>.csv
# - Prompts for test name (accepts 'test-1' or 'test-1.csv')
# - Tolerates header variants: time/timestamp/t/seconds/ms and force_N/force_kgf/force/kgf
# - Converts HH:MM:SS -> seconds (handles midnight wrap); numeric seconds and datetimes also supported
# - Computes max force, impulse (area under curve), optional energy (if speed given)
# - Plots N on the left axis; optional kgf on the right axis
# - Saves PDF next to CSV

import os, sys, re, csv
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

DATA_DIR = Path("data")
N_PER_KGF = 9.80665

# ---------- Helpers ----------
def list_csvs():
    if not DATA_DIR.exists():
        return []
    return sorted(DATA_DIR.glob("*.csv"), key=os.path.getmtime, reverse=True)

def suggest_from_last():
    files = list_csvs()
    if not files:
        return "test-"
    stem = files[0].stem
    base = re.sub(r"\d+$", "", stem) or (stem[:-1] if stem else "test-")
    return base

def _parse_hhmmss_to_sec_list(hms_list):
    """Convert ['HH:MM:SS(.sss)', ...] to seconds since first point, handling midnight wrap."""
    secs = []
    base = None
    accum_day = 0.0
    last_abs = None
    for s in hms_list:
        s = s.strip()
        try:
            hh, mm, ss = s.split(":")
            x = int(hh) * 3600 + int(mm) * 60 + float(ss)
        except Exception:
            secs.append(float("nan"))
            continue
        if base is None:
            base = x
        if last_abs is not None and x < last_abs:
            accum_day += 24 * 3600  # crossed midnight
        secs.append((x - base) + accum_day)
        last_abs = x
    return secs

def _to_float_array(vals):
    out = []
    for v in vals:
        try:
            out.append(float(v))
        except Exception:
            out.append(float("nan"))
    return np.array(out, dtype=float)

def integrate_trapezoid(y, x):
    """Stable trapezoid integral that ignores non-increasing time steps."""
    y = np.asarray(y, dtype=float)
    x = np.asarray(x, dtype=float)
    if y.size < 2 or x.size < 2:
        return 0.0
    area = 0.0
    for i in range(len(x) - 1):
        dt = x[i+1] - x[i]
        if dt > 0:
            area += 0.5 * (y[i] + y[i+1]) * dt
    return area

def finite_xy(x, y):
    m = np.isfinite(x) & np.isfinite(y)
    return x[m], y[m]

# ---------- CSV loader (robust to variants) ----------
def read_logger_csv(path: Path):
    """
    Load CSV with typical columns written by read_force.py:
      - time (HH:MM:SS) or numeric seconds (or 'timestamp', 't', 'seconds', 'ms')
      - force_N or force_kgf (or 'force', 'kgf')
      - optional 'sample'
    Returns: t_seconds (np.array), F_N (np.array), sample_name (str)
    """
    if not path.exists():
        raise FileNotFoundError(path)

    # Try to sniff the dialect; fall back to comma
    try:
        with path.open("r", encoding="utf-8") as f:
            preview = f.read(2048)
        dialect = csv.Sniffer().sniff(preview, delimiters=",;\t ")
        has_header = csv.Sniffer().has_header(preview)
    except Exception:
        dialect = csv.get_dialect("excel")
        has_header = True

    with path.open("r", encoding="utf-8") as f:
        rdr = csv.reader(f, dialect)
        rows = list(rdr)

    if not rows:
        raise RuntimeError("CSV appears empty.")

    # Headers
    if has_header:
        headers = [c.strip().lower() for c in rows[0]]
        data_rows = rows[1:]
    else:
        # Assume default header if not present
        headers = ["time", "force_n"]
        data_rows = rows

    # Build name → index map
    idx = {name: i for i, name in enumerate(headers)}

    # Find time-like and force-like columns
    time_candidates = ["time", "timestamp", "t", "datetime", "seconds", "ms"]
    forceN_candidates = ["force_n", "forcen", "f_n", "n", "force"]
    forceK_candidates = ["force_kgf", "forcekgf", "f_kgf", "kgf"]

    def find_col(cands):
        for c in cands:
            if c in idx:
                return idx[c], c
        return None, None

    ti, tname = find_col(time_candidates)
    if ti is None:
        raise RuntimeError("No time-like column found (need one of: time/timestamp/t/datetime/seconds/ms).")

    ni, nname = find_col(forceN_candidates)
    ki, kname = find_col(forceK_candidates)

    if ni is None and ki is None:
        raise RuntimeError("No force column found (need one of: force_N/force_kgf/force/kgf).")

    si, _ = find_col(["sample", "name", "test", "test_name"])

    # Extract columns as lists of strings (guarding short rows)
    def col(i):
        return [row[i].strip() if i is not None and i < len(row) else "" for row in data_rows]

    t_raw = col(ti)
    F_N = None

    # Time parsing:
    # 1) If numeric, use directly as seconds
    # 2) If looks like HH:MM:SS(.sss), convert with wrap handling
    # 3) If other date/time string, try fromisoformat, else NaN
    numeric_time = True
    for s in t_raw:
        try:
            float(s)
        except Exception:
            numeric_time = False
            break

    if numeric_time:
        t_sec = _to_float_array(t_raw)
    else:
        # HH:MM:SS?
        if all((":" in s) for s in t_raw if s):
            t_sec = np.array(_parse_hhmmss_to_sec_list(t_raw), dtype=float)
            # If all NaN (unlikely), fall through to ISO attempts
            if np.all(~np.isfinite(t_sec)):
                # Try ISO/datetime
                dt_vals = []
                for s in t_raw:
                    s = s.replace("Z", "+00:00")
                    try:
                        dt = datetime.fromisoformat(s)
                        dt_vals.append(dt)
                    except Exception:
                        dt_vals.append(None)
                if all(v is None for v in dt_vals):
                    raise RuntimeError("Could not parse 'time' column.")
                t0 = next(v for v in dt_vals if v is not None)
                t_sec = np.array([(v - t0).total_seconds() if v else np.nan for v in dt_vals], dtype=float)
        else:
            # ISO/datetime
            dt_vals = []
            for s in t_raw:
                s = s.replace("Z", "+00:00")
                try:
                    dt = datetime.fromisoformat(s)
                    dt_vals.append(dt)
                except Exception:
                    # last resort: treat as HH:MM:SS without date
                    if ":" in s:
                        dt_vals.append(None)
                    else:
                        dt_vals.append(None)
            if all(v is None for v in dt_vals):
                # fallback to HH:MM:SS converter anyway
                t_sec = np.array(_parse_hhmmss_to_sec_list(t_raw), dtype=float)
            else:
                t0 = next(v for v in dt_vals if v is not None)
                t_sec = np.array([(v - t0).total_seconds() if v else np.nan for v in dt_vals], dtype=float)

    # Force parsing: prefer N if present; else kgf → N
    if ni is not None:
        F_N = _to_float_array(col(ni))
    elif ki is not None:
        F_N = _to_float_array(col(ki)) * N_PER_KGF

    sample = None
    if si is not None:
        svals = col(si)
        if svals:
            sample = svals[0] or path.stem
    if not sample:
        sample = path.stem

    return t_sec, F_N, sample

# ---------- Main ----------
def main():
    # Prompt for test name (smart default from latest CSV)
    default = suggest_from_last()
    name = input(f"Enter test name [{default}]: ").strip() or default
    if name.lower().endswith(".csv"):
        name = name[:-4]
    csv_path = DATA_DIR / f"{name}.csv"
    if not csv_path.exists():
        # keep output minimal but helpful
        print(f"File not found: {csv_path}")
        recent = [p.name for p in list_csvs()[:8]]
        if recent:
            print("Recent CSVs:")
            for r in recent:
                print("  -", r)
        sys.exit(1)

    # Load
    t, F_N, sample = read_logger_csv(csv_path)
    t, F_N = finite_xy(t, F_N)
    if t.size < 2:
        print("Not enough valid points to plot.")
        sys.exit(1)

    # Stats
    duration = float(t[-1] - t[0])
    diffs = np.diff(t)
    diffs = diffs[diffs > 0]
    hz_est = (1.0 / np.median(diffs)) if diffs.size else float("nan")
    idx_max = int(np.nanargmax(F_N))
    t_max, FmaxN = float(t[idx_max]), float(F_N[idx_max])
    FmaxK = FmaxN / N_PER_KGF

    # Areas & energy
    impulse_Ns = integrate_trapezoid(F_N, t)          # N·s
    area_kgf_s = impulse_Ns / N_PER_KGF               # kgf·s

    speed_input = input("Enter crosshead speed in mm/s (blank to skip energy): ").strip()
    energy_J = None
    travel_mm = None
    if speed_input:
        try:
            v_mm_s = float(speed_input)
            if v_mm_s > 0:
                v_m_s = v_mm_s / 1000.0
                energy_J = integrate_trapezoid(F_N * v_m_s, t)  # J ≈ ∫ F v dt
                travel_mm = v_mm_s * duration
        except Exception:
            pass

    # ---------- Make the PDF (one page) ----------
    pdf_path = csv_path.with_suffix(".pdf")
    fig = plt.figure(figsize=(8.5, 11.0), dpi=150)  # US Letter

    # Main plot area
    ax = fig.add_axes([0.12, 0.35, 0.83, 0.55])
    ax.plot(t, F_N, linewidth=1.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.3)

    # Optional right axis in kgf
    ax_r = ax.twinx()
    ax_r.set_ylabel("Force (kgf)")
    # Sync the right axis ticks to N→kgf
    ymin, ymax = ax.get_ylim()
    ax_r.set_ylim(ymin / N_PER_KGF, ymax / N_PER_KGF)

    # Max marker
    ax.plot([t_max], [FmaxN], marker="o")
    ax.annotate(f"Max = {FmaxN:.3f} N ({FmaxK:.3f} kgf)",
                xy=(t_max, FmaxN), xytext=(10, 10),
                textcoords="offset points", arrowprops=dict(arrowstyle="->"))

    # Title
    title = f"Force vs Time — {sample}"
    fig.suptitle(title, fontsize=16, y=0.97)

    # Info panel (bottom)
    ax2 = fig.add_axes([0.08, 0.10, 0.84, 0.18]); ax2.axis("off")
    lines = [
        f"CSV: {csv_path.name}",
        f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"Duration: {duration:.3f} s    Est. rate: {hz_est:.2f} Hz",
        f"Max force: {FmaxN:.3f} N ({FmaxK:.3f} kgf)",
        f"Area under curve (impulse): {impulse_Ns:.3f} N·s  ({area_kgf_s:.3f} kgf·s)",
    ]
    if energy_J is not None:
        lines.append(f"Estimated work: {energy_J:.3f} J  (speed assumed constant)")
        if travel_mm is not None:
            lines.append(f"Assumed speed: {v_mm_s:.3f} mm/s   Estimated travel: {travel_mm:.2f} mm")
    ax2.text(0.02, 0.98, "\n".join(lines), va="top", fontsize=10)

    fig.text(0.5, 0.02, "Generated by output_data.py", ha="center", fontsize=9)

    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(pdf_path) as pdf:
        pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)

    print(f"Report saved: {pdf_path}")

if __name__ == "__main__":
    main()
