#!/usr/bin/env python3
import argparse, csv, math, os, sys, time
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless-safe
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def parse_hms_to_seconds(hms_list):
    # Convert ["HH:MM:SS", ...] to seconds from start; handle midnight wrap.
    secs = []
    base = None
    day_offset = 0.0
    last = None
    for t in hms_list:
        try:
            h, m, s = t.strip().split(":")
            x = int(h)*3600 + int(m)*60 + float(s)
        except Exception:
            x = float("nan")
        if base is None and math.isfinite(x):
            base = x
        if last is not None and math.isfinite(x) and math.isfinite(last) and x < last:
            # wrapped past midnight
            day_offset += 24*3600
        if math.isfinite(x) and base is not None:
            secs.append((x - base) + day_offset)
        else:
            secs.append(float("nan"))
        last = x
    return secs

def load_csv(path: Path):
    # Expected columns: time, force_N, (optional) force_kgf, sample
    times, FN, KG, sample_name = [], [], [], ""
    with path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        gotN = "force_N" in r.fieldnames if r.fieldnames else False
        gotK = "force_kgf" in r.fieldnames if r.fieldnames else False
        for row in r:
            times.append(row.get("time","").strip())
            if gotN: FN.append(float(row["force_N"]))
            elif gotK: FN.append(float(row["force_kgf"]) * 9.80665)  # fallback
            else: raise ValueError("CSV missing force_N / force_kgf")
            if gotK: KG.append(float(row["force_kgf"]))
            sample_name = row.get("sample", sample_name)
    t_sec = parse_hms_to_seconds(times)
    return t_sec, FN, KG if KG else None, sample_name or "sample"

def finite_pairs(t, y):
    out_t, out_y = [], []
    for a,b in zip(t,y):
        if math.isfinite(a) and math.isfinite(b):
            out_t.append(a); out_y.append(b)
    return out_t, out_y

def estimate_dt(t):
    if len(t) < 2: return 0.0
    diffs = [t[i+1]-t[i] for i in range(len(t)-1) if t[i+1]>t[i]]
    if not diffs: return 0.0
    diffs.sort()
    return diffs[len(diffs)//2]  # median

def integrate_trapz(y, t):
    # trapezoidal ∫ y dt
    if len(y) < 2: return 0.0
    area = 0.0
    for i in range(len(y)-1):
        dt = t[i+1] - t[i]
        if dt > 0:
            area += 0.5 * (y[i] + y[i+1]) * dt
    return area

def nice_filename(base: str) -> str:
    safe = "".join(c if c.isalnum() or c in (".","_","-") else "_" for c in base).strip("_")
    return safe or "report"

def main():
    p = argparse.ArgumentParser(description="Make a 1-page PDF report from force CSV")
    p.add_argument("--csv", required=True, help="path to CSV from read_force.py")
    p.add_argument("--out", default="", help="output PDF path (default beside CSV)")
    p.add_argument("--title", default="", help="report title override")
    p.add_argument("--speed-mm-s", type=float, default=None,
                   help="constant crosshead speed (mm/s) to estimate work (J)")
    p.add_argument("--show-kgf", action="store_true", help="annotate max in kgf too")
    args = p.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}", file=sys.stderr); sys.exit(1)

    t_raw, F_N_raw, KG_raw, sample = load_csv(csv_path)
    t, F = finite_pairs(t_raw, F_N_raw)
    if len(t) < 2:
        print("Not enough points to plot.", file=sys.stderr); sys.exit(1)

    # Stats
    duration = t[-1] - t[0]
    dt_est = estimate_dt(t)
    hz_est = (1.0/dt_est) if dt_est>0 else float("nan")
    i_max = max(range(len(F)), key=lambda i: F[i])
    t_max, Fmax = t[i_max], F[i_max]
    J_impulse = integrate_trapz(F, t)  # N*s
    energy_J = None
    distance_m = None
    if args.speed_mm_s is not None and math.isfinite(args.speed_mm_s) and args.speed_mm_s > 0:
        v = args.speed_mm_s / 1000.0  # m/s
        energy_J = integrate_trapz([f*v for f in F], t)  # J ≈ ∫F v dt
        distance_m = v * duration

    # PDF name
    if args.out:
        pdf_path = Path(args.out)
    else:
        base = f"{csv_path.stem}_report"
        pdf_path = csv_path.with_name(nice_filename(base)+".pdf")

    # Figure (one page)
    fig = plt.figure(figsize=(8.5, 11.0), dpi=150)  # US letter
    ax = fig.add_axes([0.12, 0.35, 0.83, 0.55])     # big top plot

    # Plot force vs time
    ax.plot(t, F, linewidth=1.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.3)

    # Mark max
    ax.plot([t_max], [Fmax], marker="o")
    label = f"Max = {Fmax:.3f} N"
    if args.show_kgf:
        label += f"  ({Fmax/9.80665:.3f} kgf)"
    ax.annotate(label, xy=(t_max, Fmax), xytext=(10, 10),
                textcoords="offset points", arrowprops=dict(arrowstyle="->"))

    # Title block
    title = args.title or f"Force vs Time — {sample}"
    fig.suptitle(title, fontsize=16, y=0.97)

    # Info panel (bottom)
    ax2 = fig.add_axes([0.08, 0.10, 0.84, 0.18])
    ax2.axis("off")

    lines = []
    lines.append(f"CSV: {csv_path.name}")
    lines.append(f"Date generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"Sample: {sample}")
    lines.append(f"Duration: {duration:.3f} s    Est. rate: {hz_est:.2f} Hz")
    lines.append(f"Max force: {Fmax:.3f} N ({Fmax/9.80665:.3f} kgf)")
    lines.append(f"Area under curve (impulse): {J_impulse:.3f} N·s")
    if energy_J is not None:
        lines.append(f"Estimated work (requires constant speed): {energy_J:.3f} J")
        lines.append(f"Assumed speed: {args.speed_mm_s:.3f} mm/s  Estimated travel: {distance_m*1000:.2f} mm")

    text = "\n".join(lines)
    ax2.text(0.02, 0.98, text, va="top", fontsize=10)

    # Footer
    fig.text(0.5, 0.02, "Generated by make_report.py", ha="center", fontsize=9)

    # Save
    pdf_path.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(pdf_path) as pdf:
        pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)

    print(f"Report saved: {pdf_path}")

if __name__ == "__main__":
    main()
