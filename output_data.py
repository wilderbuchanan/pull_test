#!/usr/bin/env python3
# One-page PDF report from data/<test>.csv
# - Prompts for test name (accepts 'test-1' or 'test-1.csv')
# - Robust CSV loader: auto-detects time & force columns even if headers are missing/wrong
# - Accepts time as HH:MM:SS(.sss), ISO datetime, or numeric seconds (handles midnight wrap)
# - Computes max force, impulse (area under curve), optional energy (with constant speed)
# - Left axis: N, Right axis: kgf
# - Minimal console chatter

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

# ---------- Utils ----------
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

def _parse_hhmmss_list_to_sec(hms_list):
    secs, base, accum_day, last_abs = [], None, 0.0, None
    for s in hms_list:
        s = s.strip()
        try:
            hh, mm, ss = s.split(":")
            x = int(hh) * 3600 + int(mm) * 60 + float(ss)
        except Exception:
            secs.append(float("nan")); continue
        if base is None: base = x
        if last_abs is not None and x < last_abs: accum_day += 24 * 3600
        secs.append((x - base) + accum_day)
        last_abs = x
    return np.array(secs, dtype=float)

def _to_float_array(vals):
    out = []
    for v in vals:
        try: out.append(float(v))
        except Exception: out.append(float("nan"))
    return np.array(out, dtype=float)

def integrate_trapezoid(y, x):
    y, x = np.asarray(y, float), np.asarray(x, float)
    if y.size < 2 or x.size < 2: return 0.0
    area = 0.0
    for i in range(len(x)-1):
        dt = x[i+1] - x[i]
        if dt > 0: area += 0.5 * (y[i] + y[i+1]) * dt
    return area

def finite_xy(x, y):
    m = np.isfinite(x) & np.isfinite(y)
    return x[m], y[m]

# ---------- Column auto-detection ----------
_HMS_RE = re.compile(r"^\s*\d{1,2}:\d{2}:\d{2}(?:\.\d+)?\s*$")

def _looks_hms(s): return bool(_HMS_RE.match(s or ""))

def _looks_iso(s):
    if not s: return False
    s = s.strip().replace("Z", "+00:00")
    try:
        datetime.fromisoformat(s)
        return True
    except Exception:
        return False

def _col_stats(col):
    """Return dict with heuristics for a column of strings."""
    n = len(col) if col is not None else 0
    if n == 0:
        return dict(n=0, frac_num=0, frac_hms=0, frac_iso=0, inc_frac=0, std=0, numeric=np.array([]))
    # numeric parsing
    num = _to_float_array(col)
    is_num = np.isfinite(num)
    frac_num = float(is_num.mean()) if n else 0.0
    # time-like shapes
    frac_hms = float(sum(_looks_hms(s) for s in col))/n
    frac_iso = float(sum(_looks_iso(s) for s in col))/n
    # monotonic increase (for numeric seconds)
    inc = []
    for i in range(len(num)-1):
        if np.isfinite(num[i]) and np.isfinite(num[i+1]):
            inc.append(1.0 if (num[i+1] - num[i]) >= 0 else 0.0)
    inc_frac = float(np.mean(inc)) if inc else 0.0
    # spread for force-like numeric columns
    std = float(np.nanstd(num)) if np.any(is_num) else 0.0
    return dict(n=n, frac_num=frac_num, frac_hms=frac_hms, frac_iso=frac_iso,
                inc_frac=inc_frac, std=std, numeric=num)

def _choose_time_force(columns, headers_lower):
    """
    columns: list of lists (strings) per column
    headers_lower: list of lowercased header names (or synthetic names if no header)
    Returns indices: (ti, ni, ki, si) for time, force_N, force_kgf, sample (may be None)
    """
    idx = {name:i for i, name in enumerate(headers_lower)}
    # header-based first pass
    time_names = ["time","timestamp","t","datetime","seconds","ms"]
    forceN_names = ["force_n","forcen","f_n","n","force"]
    forceK_names = ["force_kgf","forcekgf","f_kgf","kgf"]
    sample_names = ["sample","name","test","test_name"]

    def find_any(names):
        for nm in names:
            if nm in idx: return idx[nm]
        return None

    ti = find_any(time_names)
    ni = find_any(forceN_names)
    ki = find_any(forceK_names)
    si = find_any(sample_names)

    # If time not found, auto-detect by heuristics
    if ti is None:
        stats = [ _col_stats(columns[i]) for i in range(len(columns)) ]
        # Prefer HH:MM:SS (.frac) looking column
        cand_hms = [ (i, st["frac_hms"]) for i, st in enumerate(stats) ]
        cand_hms.sort(key=lambda x: x[1], reverse=True)
        if cand_hms and cand_hms[0][1] >= 0.5:
            ti = cand_hms[0][0]
        else:
            # Prefer ISO datetime column
            cand_iso = [ (i, st["frac_iso"]) for i, st in enumerate(stats) ]
            cand_iso.sort(key=lambda x: x[1], reverse=True)
            if cand_iso and cand_iso[0][1] >= 0.4:
                ti = cand_iso[0][0]
            else:
                # Prefer numeric & mostly non-decreasing
                cand_num_inc = [ (i, st["frac_num"]*0.6 + st["inc_frac"]*0.4) for i, st in enumerate(stats) ]
                cand_num_inc.sort(key=lambda x: x[1], reverse=True)
                if cand_num_inc and cand_num_inc[0][1] >= 0.5:
                    ti = cand_num_inc[0][0]
                else:
                    # Last resort: first column
                    ti = 0

    # If neither N nor kgf found, pick the most “force-like” numeric column (highest std) not equal to time
    if ni is None and ki is None:
        stats = [ _col_stats(columns[i]) for i in range(len(columns)) ]
        # zero out time column score
        best_i, best_score = None, -1.0
        for i, st in enumerate(stats):
            if i == ti: continue
            score = st["frac_num"] * (1.0 + st["std"])
            if score > best_score:
                best_i, best_score = i, score
        ni = best_i  # treat as Newtons if units unknown

    return ti, ni, ki, si

# ---------- CSV loader ----------
def read_logger_csv(path: Path):
    """
    Load CSV robustly and return (t_seconds, F_N, sample_name).
    Accepts:
      - time: HH:MM:SS(.sss), ISO datetime, or numeric seconds
      - force: N or kgf (if no units, treated as N)
      - optional 'sample' column
    """
    if not path.exists():
        raise FileNotFoundError(path)

    # sniff dialect
    try:
        with path.open("r", encoding="utf-8") as f:
            preview = f.read(4096)
        dialect = csv.Sniffer().sniff(preview, delimiters=",;\t ")
        has_header = csv.Sniffer().has_header(preview)
    except Exception:
        dialect = csv.get_dialect("excel")
        has_header = True

    with path.open("r", encoding="utf-8") as f:
        rdr = csv.reader(f, dialect)
        rows = [row for row in rdr]

    if not rows:
        raise RuntimeError("CSV appears empty.")

    # headers
    if has_header:
        raw_headers = [c.strip() for c in rows[0]]
        headers_lower = [h.lower() for h in raw_headers]
        data_rows = rows[1:]
    else:
        # synthesize headers
        width = max(len(r) for r in rows)
        raw_headers = [f"col{i+1}" for i in range(width)]
        headers_lower = raw_headers[:]
        data_rows = rows

    # normalize row widths
    width = len(headers_lower)
    norm_rows = []
    for r in data_rows:
        if len(r) < width:
            r = r + [""]*(width-len(r))
        norm_rows.append(r)

    # columns as lists of strings
    columns = [ [r[i] for r in norm_rows] for i in range(width) ]

    # choose columns
    ti, ni, ki, si = _choose_time_force(columns, [h.lower() for h in headers_lower])

    # extract raw columns
    t_raw = columns[ti] if ti is not None else []
    sample = None
    if si is not None and si < len(columns):
        svals = [s for s in columns[si] if s]
        sample = svals[0] if svals else None
    if not sample:
        sample = path.stem

    # parse time
    # decide shape
    if all((_looks_hms(s) or not s) for s in t_raw):
        t_sec = _parse_hhmmss_list_to_sec(t_raw)
    elif all((_looks_iso(s) or not s) for s in t_raw):
        dt_vals = []
        for s in t_raw:
            s = (s or "").strip().replace("Z", "+00:00")
            try: dt_vals.append(datetime.fromisoformat(s))
            except Exception: dt_vals.append(None)
        t0 = next((v for v in dt_vals if v is not None), None)
        t_sec = np.array([(v - t0).total_seconds() if v else np.nan for v in dt_vals], dtype=float) if t0 else np.full(len(t_raw), np.nan)
    else:
        # try numeric seconds; if many fail, fallback to HH:MM:SS
        t_sec = _to_float_array(t_raw)
        if np.isnan(t_sec).mean() > 0.4:  # too many NaN, try HH:MM:SS
            t_sec = _parse_hhmmss_list_to_sec(t_raw)

    # parse force
    if ni is not None:
        F_N = _to_float_array(columns[ni])
    elif ki is not None:
        F_N = _to_float_array(columns[ki]) * N_PER_KGF
    else:
        raise RuntimeError("Could not locate a force column.")

    return t_sec, F_N, sample

# ---------- Main ----------
def main():
    default = suggest_from_last()
    name = input(f"Enter test name [{default}]: ").strip() or default
    if name.lower().endswith(".csv"): name = name[:-4]
    csv_path = DATA_DIR / f"{name}.csv"
    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        recent = [p.name for p in list_csvs()[:8]]
        if recent:
            print("Recent CSVs:"); [print("  -", r) for r in recent]
        sys.exit(1)

    # Load & clean
    t, F_N, sample = read_logger_csv(csv_path)
    t, F_N = finite_xy(t, F_N)
    if t.size < 2:
        print("Not enough valid points to plot."); sys.exit(1)

    # Stats
    duration = float(t[-1] - t[0])
    diffs = np.diff(t); diffs = diffs[diffs > 0]
    hz_est = (1.0 / np.median(diffs)) if diffs.size else float("nan")
    idx_max = int(np.nanargmax(F_N)); t_max, FmaxN = float(t[idx_max]), float(F_N[idx_max])
    FmaxK = FmaxN / N_PER_KGF

    # Areas / energy
    impulse_Ns = integrate_trapezoid(F_N, t)          # N·s
    area_kgf_s = impulse_Ns / N_PER_KGF               # kgf·s

    speed_input = input("Enter crosshead speed in mm/s (blank to skip energy): ").strip()
    energy_J = None; travel_mm = None
    if speed_input:
        try:
            v_mm_s = float(speed_input)
            if v_mm_s > 0:
                v_m_s = v_mm_s / 1000.0
                energy_J = integrate_trapezoid(F_N * v_m_s, t)
                travel_mm = v_mm_s * duration
        except Exception:
            pass

    # ---------- PDF ----------
    pdf_path = csv_path.with_suffix(".pdf")
    fig = plt.figure(figsize=(8.5, 11.0), dpi=150)  # US Letter
    ax = fig.add_axes([0.12, 0.35, 0.83, 0.55])
    ax.plot(t, F_N, linewidth=1.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.grid(True, alpha=0.3)

    # Right axis (kgf)
    ax_r = ax.twinx()
    ax_r.set_ylabel("Force (kgf)")
    ymin, ymax = ax.get_ylim()
    ax_r.set_ylim(ymin / N_PER_KGF, ymax / N_PER_KGF)

    # Max marker
    ax.plot([t_max], [FmaxN], marker="o")
    ax.annotate(f"Max = {FmaxN:.3f} N ({FmaxK:.3f} kgf)", xy=(t_max, FmaxN),
                xytext=(10, 10), textcoords="offset points",
                arrowprops=dict(arrowstyle="->"))

    # Title
    fig.suptitle(f"Force vs Time — {sample}", fontsize=16, y=0.97)

    # Info panel
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
