#!/usr/bin/env python3
"""
Summarize all test CSVs in data/ into one PDF (no graphs).

For each CSV, computes:
- points, duration, estimated sample rate
- max force (N & kgf), mean force (N)
- area under curve (impulse) in N·s and kgf·s
- optional energy (J) if crosshead speed is given (assumes constant speed)

Usage:
  python summarize_tests.py [--dir data] [--out summary.pdf] [--speed-mm-s 5.0] [--sort mtime|name]
"""

import os, sys, re, csv, argparse
from pathlib import Path
from datetime import datetime

import numpy as np

# -------- PDF (ReportLab) --------
from reportlab.platypus import SimpleDocTemplate, Paragraph, Table, TableStyle, Spacer, PageBreak
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet

DATA_DIR_DEFAULT = Path("data")
N_PER_KGF = 9.80665

# ---------- small utils ----------
def human_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")

def finite_xy(x, y):
    m = np.isfinite(x) & np.isfinite(y)
    return x[m], y[m]

def integrate_trapezoid(y, x):
    y, x = np.asarray(y, float), np.asarray(x, float)
    if y.size < 2 or x.size < 2:
        return 0.0
    area = 0.0
    for i in range(len(x)-1):
        dt = x[i+1] - x[i]
        if dt > 0:
            area += 0.5 * (y[i] + y[i+1]) * dt
    return area

# -------- robust CSV loader (auto-detect time/force) --------
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

def _parse_hhmmss_list_to_sec(hms_list):
    secs, base, accum_day, last_abs = [], None, 0.0, None
    for s in hms_list:
        s = (s or "").strip()
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

def _col_stats(col):
    n = len(col) if col is not None else 0
    if n == 0:
        return dict(n=0, frac_num=0, frac_hms=0, frac_iso=0, inc_frac=0, std=0, numeric=np.array([]))
    num = _to_float_array(col)
    is_num = np.isfinite(num)
    frac_num = float(is_num.mean()) if n else 0.0
    frac_hms = float(sum(_looks_hms(s) for s in col))/n
    frac_iso = float(sum(_looks_iso(s) for s in col))/n
    inc = []
    for i in range(len(num)-1):
        if np.isfinite(num[i]) and np.isfinite(num[i+1]):
            inc.append(1.0 if (num[i+1] - num[i]) >= 0 else 0.0)
    inc_frac = float(np.mean(inc)) if inc else 0.0
    std = float(np.nanstd(num)) if np.any(is_num) else 0.0
    return dict(n=n, frac_num=frac_num, frac_hms=frac_hms, frac_iso=frac_iso,
                inc_frac=inc_frac, std=std, numeric=num)

def _choose_time_force(columns, headers_lower):
    idx = {name:i for i, name in enumerate(headers_lower)}
    time_names  = ["time","timestamp","t","datetime","seconds","ms"]
    forceN_names= ["force_n","forcen","f_n","n","force"]
    forceK_names= ["force_kgf","forcekgf","f_kgf","kgf"]
    sample_names= ["sample","name","test","test_name"]

    def find_any(names):
        for nm in names:
            if nm in idx: return idx[nm]
        return None

    ti = find_any(time_names)
    ni = find_any(forceN_names)
    ki = find_any(forceK_names)
    si = find_any(sample_names)

    if ti is None:
        stats = [ _col_stats(columns[i]) for i in range(len(columns)) ]
        cand_hms = sorted(((i, st["frac_hms"]) for i, st in enumerate(stats)), key=lambda x: x[1], reverse=True)
        if cand_hms and cand_hms[0][1] >= 0.5:
            ti = cand_hms[0][0]
        else:
            cand_iso = sorted(((i, st["frac_iso"]) for i, st in enumerate(stats)), key=lambda x: x[1], reverse=True)
            if cand_iso and cand_iso[0][1] >= 0.4:
                ti = cand_iso[0][0]
            else:
                cand_num_inc = sorted(((i, st["frac_num"]*0.6 + st["inc_frac"]*0.4) for i, st in enumerate(stats)), key=lambda x: x[1], reverse=True)
                ti = cand_num_inc[0][0] if cand_num_inc else 0

    if ni is None and ki is None:
        stats = [ _col_stats(columns[i]) for i in range(len(columns)) ]
        best_i, best_score = None, -1.0
        for i, st in enumerate(stats):
            if i == ti: continue
            score = st["frac_num"] * (1.0 + st["std"])
            if score > best_score:
                best_i, best_score = i, score
        ni = best_i
    return ti, ni, ki, si

def read_logger_csv(path: Path):
    """
    Load CSV robustly and return (t_seconds, F_N, sample_name).
    Accepts:
      - time: HH:MM:SS(.sss), ISO datetime, or numeric seconds
      - force: N or kgf (if no units, treated as N)
      - optional 'sample' column
    """
    # sniff
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
        rows = [row for row in rdr if row and any(cell.strip() for cell in row)]

    if not rows:
        raise RuntimeError("CSV appears empty.")

    if has_header:
        raw_headers = [c.strip() for c in rows[0]]
        headers_lower = [h.lower() for h in raw_headers]
        data_rows = rows[1:]
    else:
        width = max(len(r) for r in rows)
        raw_headers = [f"col{i+1}" for i in range(width)]
        headers_lower = raw_headers[:]
        data_rows = rows

    width = len(headers_lower)
    norm_rows = []
    for r in data_rows:
        if len(r) < width:
            r = r + [""]*(width-len(r))
        norm_rows.append(r)

    columns = [ [r[i] for r in norm_rows] for i in range(width) ]
    ti, ni, ki, si = _choose_time_force(columns, headers_lower)

    t_raw = columns[ti] if ti is not None else []
    sample = None
    if si is not None and si < len(columns):
        svals = [s for s in columns[si] if s]
        sample = svals[0] if svals else None
    if not sample:
        sample = path.stem

    # time parse
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
        t_sec = _to_float_array(t_raw)
        if np.isnan(t_sec).mean() > 0.4:
            t_sec = _parse_hhmmss_list_to_sec(t_raw)

    # force parse
    if ni is not None:
        F_N = _to_float_array(columns[ni])
    elif ki is not None:
        F_N = _to_float_array(columns[ki]) * N_PER_KGF
    else:
        raise RuntimeError("Could not locate a force column.")

    return t_sec, F_N, sample

# -------- summarizer --------
def summarize_csv(csv_path: Path, speed_mm_s: float | None):
    # load
    t, F_N, sample = read_logger_csv(csv_path)
    t, F_N = finite_xy(t, F_N)
    if t.size < 2:
        return dict(file=csv_path.name, sample=sample, points=t.size, duration=np.nan,
                    hz=np.nan, maxN=np.nan, maxkgf=np.nan, meanN=np.nan,
                    impulseNs=np.nan, impulsegks=np.nan, energyJ=np.nan)

    duration = float(t[-1] - t[0])
    diffs = np.diff(t); diffs = diffs[diffs > 0]
    hz_est = (1.0 / np.median(diffs)) if diffs.size else float("nan")
    maxN = float(np.nanmax(F_N))
    maxkgf = maxN / N_PER_KGF
    meanN = float(np.nanmean(F_N))
    impulse_Ns = integrate_trapezoid(F_N, t)
    impulsegks = impulse_Ns / N_PER_KGF
    energyJ = float("nan")
    if speed_mm_s and speed_mm_s > 0:
        v_m_s = speed_mm_s / 1000.0
        energyJ = integrate_trapezoid(F_N * v_m_s, t)

    return dict(
        file=csv_path.name, sample=sample, points=int(t.size),
        duration=duration, hz=hz_est, maxN=maxN, maxkgf=maxkgf, meanN=meanN,
        impulseNs=impulse_Ns, impulsegks=impulsegks, energyJ=energyJ
    )

def build_pdf(rows, out_path: Path, speed_mm_s: float | None, src_dir: Path):
    styles = getSampleStyleSheet()
    doc = SimpleDocTemplate(str(out_path), pagesize=letter,
                            leftMargin=36, rightMargin=36, topMargin=36, bottomMargin=36)

    story = []
    title = f"Mechanical Test Summary ({len(rows)} file{'s' if len(rows)!=1 else ''})"
    story.append(Paragraph(title, styles["Title"]))
    story.append(Paragraph(f"Source folder: {src_dir.resolve()}", styles["Normal"]))
    story.append(Paragraph(f"Generated: {human_time(datetime.now().timestamp())}", styles["Normal"]))
    if speed_mm_s:
        story.append(Paragraph(f"Energy computed with constant crosshead speed = {speed_mm_s:.3f} mm/s", styles["Italic"]))
    story.append(Spacer(1, 12))

    # Build table data
    headers = ["Sample", "File", "Points", "Duration (s)", "Est. Hz",
               "Max (N)", "Max (kgf)", "Mean (N)", "Impulse (N·s)", "Impulse (kgf·s)"]
    if speed_mm_s:
        headers.append("Energy (J)")

    data = [headers]
    for r in rows:
        row = [
            r["sample"],
            r["file"],
            f'{r["points"]}',
            f'{r["duration"]:.3f}' if np.isfinite(r["duration"]) else "—",
            f'{r["hz"]:.2f}' if np.isfinite(r["hz"]) else "—",
            f'{r["maxN"]:.3f}' if np.isfinite(r["maxN"]) else "—",
            f'{r["maxkgf"]:.3f}' if np.isfinite(r["maxkgf"]) else "—",
            f'{r["meanN"]:.3f}' if np.isfinite(r["meanN"]) else "—",
            f'{r["impulseNs"]:.3f}' if np.isfinite(r["impulseNs"]) else "—",
            f'{r["impulsegks"]:.3f}' if np.isfinite(r["impulsegks"]) else "—",
        ]
        if speed_mm_s:
            row.append(f'{r["energyJ"]:.3f}' if np.isfinite(r["energyJ"]) else "—")
        data.append(row)

    col_widths = [90, 120, 45, 70, 55, 62, 62, 60, 70, 80] + ([70] if speed_mm_s else [])

    table = Table(data, repeatRows=1, colWidths=col_widths)
    table.setStyle(TableStyle([
        ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"),
        ("FONTSIZE", (0,0), (-1,0), 9),
        ("ALIGN", (2,1), (-1,-1), "RIGHT"),
        ("ALIGN", (0,0), (-1,0), "CENTER"),
        ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.whitesmoke, colors.white]),
        ("GRID", (0,0), (-1,-1), 0.25, colors.lightgrey),
        ("LEFTPADDING", (0,0), (-1,-1), 4),
        ("RIGHTPADDING", (0,0), (-1,-1), 4),
        ("TOPPADDING", (0,0), (-1,-1), 3),
        ("BOTTOMPADDING", (0,0), (-1,-1), 3),
    ]))

    story.append(table)
    doc.build(story)

# -------- main --------
def main():
    ap = argparse.ArgumentParser(description="Summarize all test CSVs into a single PDF (no graphs).")
    ap.add_argument("--dir", default=str(DATA_DIR_DEFAULT), help="Directory containing CSVs (default: data)")
    ap.add_argument("--out", default="summary.pdf", help="Output PDF path (default: summary.pdf)")
    ap.add_argument("--speed-mm-s", type=float, default=None, help="Crosshead speed in mm/s (optional, for energy)")
    ap.add_argument("--sort", choices=["mtime","name"], default="mtime", help="Sort rows by file mtime or name")
    args = ap.parse_args()

    data_dir = Path(args.dir)
    if not data_dir.exists():
        print(f"No such directory: {data_dir}")
        sys.exit(1)

    csvs = sorted(data_dir.glob("*.csv"), key=(os.path.getmtime if args.sort=="mtime" else lambda p: p.name.lower()), reverse=True if args.sort=="mtime" else False)
    if not csvs:
        print(f"No CSVs found in {data_dir}")
        sys.exit(1)

    rows = []
    for p in csvs:
        try:
            rows.append(summarize_csv(p, args.speed_mm_s))
        except Exception as e:
            rows.append(dict(
                file=p.name, sample=p.stem, points=0, duration=np.nan, hz=np.nan,
                maxN=np.nan, maxkgf=np.nan, meanN=np.nan, impulseNs=np.nan,
                impulsegks=np.nan, energyJ=np.nan
            ))

    out_path = Path(args.out)
    if out_path.is_dir():
        out_path = out_path / "summary.pdf"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    build_pdf(rows, out_path, args.speed_mm_s, data_dir)
    print(f"Summary saved: {out_path}")

if __name__ == "__main__":
    main()
