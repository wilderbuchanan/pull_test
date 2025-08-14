#!/usr/bin/env python3
# Produces a clean 1-page PDF from data/<test>.csv (time,force_N,force_kgf,sample)
# - Prompts for test name (accepts 'test-1' or 'test-1.csv')
# - Converts HH:MM:SS -> seconds
# - Computes max force, impulse (area under curve), optional energy (if speed given)
# - Saves PDF next to CSV

import os, math, csv, sys, time, re
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

DATA_DIR = Path("data")
N_PER_KGF = 9.80665

def list_csvs():
    if not DATA_DIR.exists():
        return []
    return sorted(DATA_DIR.glob("*.csv"), key=os.path.getmtime, reverse=True)

def suggest_from_last():
    files = list_csvs()
    if not files:
        return "test-"
    stem = files[0].stem
    # strip trailing digits (so 'test-1' -> 'test-')
    base = re.sub(r"\d+$", "", stem)
    return base if base else (stem[:-1] if stem else "test-")

def parse_hms_list_to_seconds(hms_list):
    """Convert ['HH:MM:SS', ...] to seconds since first point, handling midnight wrap."""
    secs = []
    base = None
    accum_day = 0.0
    last_abs = None
    for s in hms_list:
        try:
            hh, mm, ss = s.strip().split(":")
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

def read_logger_csv(path: Path):
    """Read CSV from read_force.py. Columns: time, force_N, force_kgf, sample"""
    with path.open("r", encoding="utf-8") as f:
        rdr = csv.DictReader(f)
        times, force_N, force_kgf, sample = [], [], [], None
        has_N = "force_N" in rdr.fieldnames
        has_K = "force_kgf" in rdr.fieldnames
        if not ("time" in rdr.fieldnames and (has_N or has_K)):
            raise RuntimeError("CSV must contain 'time' and 'force_N' or 'force_kgf' columns.")
        for row in rdr:
            times.append(row.get("time", "").strip())
            if has_N and row.get("force_N", "") != "":
                try:
                    force_N.append(float(row["force_N"]))
                except:
                    force_N.append(float("nan"))
            elif has_K and row.get("force_kgf", "") != "":
                # fallback: convert kgf to N
                try:
                    force_N.append(float(row["force_kgf"]) * N_PER_KGF)
                except:
                    force_N.append(float("nan"))
            else:
                force_N.append(float("nan"))
            if has_K and row.get("force_kgf", "") != "":
                try:
                    force_kgf.append(float(row["force_kgf"]))
                except:
                    force_kgf.append(float("nan"))
            sname = row.get("sample", None)
            if sname:
                sample = sname
    t_sec = parse_hms_list_to_seconds(times)
    if not sample:
        sample = path.stem
    return np.array(t_sec, dtype=float), np.array(force_N, dtype=float), sample

def finite_xy(x, y):
    mask = np.isfinite(x) & np.isfinite(y)
    return x[mask], y[mask]

def integrate_trapz(y, x):
    # Robust trapezoid integration; avoid deprecation issues
    if len(x) < 2 or len(y) < 2:
        return 0.0
    area = 0.0
    for i in range(len(x) - 1):
        dt = x[i+1] - x[i]
        if dt > 0:
            area += 0.5 * (y[i] + y[i+1]) * dt
    return area

def main():
    # Prompt for name with smart default
    default = suggest_from_last()
    name = input(f"Enter test name [{default}]: ").strip()
    if not name:
        name = default
    if name.lower().endswith(".csv"):
        name = name[:-4]
    csv_path = DATA_DIR / f"{name}.csv"
    if not csv_path.exists():
        print(f"File not found: {csv_path}\nAvailable CSVs:")
        for p in list_csvs()[:10]:
            print("  -", p.name)
        sys.exit(1)

    # Load and clean
    t, F_N, sample = read_logger_csv(csv_path)
    t, F_N = finite_xy(t, F_N)
    if len(t) < 2:
        print("Not enough valid points to plot.")
        sys.exit(1)

    # Stats
    duration = t[-1] - t[0]
    diffs = np.diff(t)
    diffs = diffs[diffs > 0]
    hz_est = (1.0 / np.median(diffs)) if diffs.size else float("nan")
    idx_max = int(np.nanargmax(F_N))
    t_max, FmaxN = t[idx_max], F_N[idx_max]
    FmaxK = FmaxN / N_PER_KGF

    # Areas
    impulse_Ns = integrate_trapz(F_N, t)       # N·s
    area_kgf_s = impulse_Ns / N_PER_KGF        # kgf·s

    # Optional: ask for constant speed to estimate energy
    speed_input = input("Enter crosshead speed in mm/s (blank to skip energy): ").strip()
    energy_J = None
    travel_mm = None
    if speed_input:
        try:
            v_mm_s = float(speed_input)
            if v_mm_s > 0:
                v_m_s = v_mm_s / 1000.0
                energy_J = integrate_trapz(F_N * v_m_s, t)  # J ≈ ∫ F v dt
                travel_mm = v_mm_s * duration
        except:
            pass  # ignore bad input

    # Make PDF (one page)
    pdf_path = csv_path.with_suffix(".pdf")
    fig = plt.figure(figsize=(8.5, 11.0), dpi=150)  # US Letter
    ax = fig.add_axes([0.12, 0.35, 0.83, 0.55])

    # Plot
    ax.plot(t, F_N, linewidth=1.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.3)

    # Max marker
    ax.plot([t_max], [FmaxN], marker="o")
    ax.annotate(f"Max = {FmaxN:.3f} N ({FmaxK:.3f} kgf)",
                xy=(t_max, FmaxN), xytext=(10, 10),
                textcoords="offset points", arrowprops=dict(arrowstyle="->"))

    # Title
    title = f"Force vs Time — {sample}"
    fig.suptitle(title, fontsize=16, y=0.97)

    # Info panel
    ax2 = fig.add_axes([0.08, 0.10, 0.84, 0.18]); ax2.axis("off")
    lines = [
        f"CSV: {csv_path.name}",
        f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"Sample: {sample}",
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
