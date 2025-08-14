import argparse
import pandas as pd
import matplotlib.pyplot as plt
from reportlab.lib.pagesizes import letter
from reportlab.pdfgen import canvas
from pathlib import Path
import numpy as np
import os

# --- Find latest CSV and suggest next test name ---
data_dir = Path("data")
csv_files = sorted(data_dir.glob("*.csv"), key=os.path.getmtime, reverse=True)

if not csv_files:
    print("No CSV files found in 'data/'")
    exit(1)

last_file = csv_files[0]
last_name = last_file.stem  # without .csv
suggested_name = last_name[:-1] if last_name else last_name

# --- Ask user for name ---
user_name = input(f"Enter test name [{suggested_name}]: ").strip()
if not user_name:
    user_name = suggested_name

csv_path = data_dir / f"{user_name}.csv"
if not csv_path.exists():
    print(f"File '{csv_path}' not found.")
    exit(1)

# --- Load data ---
df = pd.read_csv(csv_path)
if "Force (kgf)" in df.columns:
    force_col = "Force (kgf)"
else:
    force_col = df.columns[1]

time_col = df.columns[0]

# --- Analysis ---
max_force = df[force_col].max()
area_under_curve = np.trapz(df[force_col], df[time_col])  # kgf·s

# --- Plot ---
plt.figure(figsize=(8, 5))
plt.plot(df[time_col], df[force_col], label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force (kgf)")
plt.title(user_name)
plt.grid(True)
plt.legend()
plot_path = data_dir / f"{user_name}.png"
plt.savefig(plot_path, dpi=300)
plt.close()

# --- PDF ---
pdf_path = data_dir / f"{user_name}.pdf"
c = canvas.Canvas(str(pdf_path), pagesize=letter)
width, height = letter

c.setFont("Helvetica-Bold", 16)
c.drawString(50, height - 50, f"Test Report: {user_name}")

c.setFont("Helvetica", 12)
c.drawString(50, height - 100, f"Max Force: {max_force:.2f} kgf")
c.drawString(50, height - 120, f"Area Under Curve (kgf·s): {area_under_curve:.2f}")

c.drawImage(str(plot_path), 50, height - 400, width=500, preserveAspectRatio=True)

c.showPage()
c.save()

print(f"Report saved to {pdf_path}")
