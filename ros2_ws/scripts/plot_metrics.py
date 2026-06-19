#!/usr/bin/env python3
"""Plot simulation metrics from the CSV produced by the headless CsvObserver.

Each CSV row is one simulation (config parameters + outcome metrics). This
script aggregates rows by scenario and the `always_charge_at_dock` flag (mean
over the Monte-Carlo rounds) and renders a grouped-bar comparison.

Usage:
  python3 scripts/plot_metrics.py [results/metrics.csv] [results/metrics_plot.png]
"""
import sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "results/metrics.csv"
out_path = sys.argv[2] if len(sys.argv) > 2 else "results/metrics_plot.png"

df = pd.read_csv(csv_path)

# Metrics worth comparing off vs on, with friendly titles.
metrics = [
    ("charge_cycles_total", "Charge cycles (total)"),
    ("charge_cycles_partial", "Partial charge cycles"),
    ("deep_discharge_count", "Deep discharges"),
    ("avg_depth_of_discharge", "Avg depth of discharge"),
    ("charging_time", "Charging time [s]"),
    ("scheduled_rejected", "Scheduled rejected"),
]

agg = df.groupby(["scenario", "always_charge_at_dock"]).mean(numeric_only=True)
scenarios = sorted(df["scenario"].unique())
x = range(len(scenarios))
width = 0.38

fig, axes = plt.subplots(2, 3, figsize=(16, 8))
for ax, (col, title) in zip(axes.flat, metrics):
    off = [agg.loc[(s, 0), col] if (s, 0) in agg.index else 0 for s in scenarios]
    on = [agg.loc[(s, 1), col] if (s, 1) in agg.index else 0 for s in scenarios]
    ax.bar([i - width / 2 for i in x], off, width, label="off", color="#5b8")
    ax.bar([i + width / 2 for i in x], on, width, label="always_charge_at_dock", color="#e8a")
    ax.set_title(title)
    ax.set_xticks(list(x))
    ax.set_xticklabels(scenarios, rotation=30, ha="right", fontsize=8)
    ax.legend(fontsize=8)

fig.suptitle("Battery / mission metrics by scenario: opportunistic dock charging off vs on", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(out_path, dpi=120)
print(f"wrote {out_path}")
