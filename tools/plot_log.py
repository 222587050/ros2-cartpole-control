"""Logger düğümünün yazdığı CSV'yi grafiğe çevirir.

    python3 tools/plot_log.py /tmp/cartpole_log.csv results/lqr_run.png
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main(csv_path, out_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])

    def col(k):
        return np.array([float(r[k]) for r in rows])

    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    ax[0].plot(t, np.rad2deg(col("theta")))
    ax[0].set(ylabel="Açı (derece)", title=f"Kayıt: {csv_path}")
    ax[1].plot(t, col("x"))
    ax[1].set(ylabel="Araba konumu (m)")
    ax[2].plot(t, col("force"))
    ax[2].set(ylabel="Kuvvet (N)", xlabel="Zaman (s)")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Kaydedildi: {out_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Kullanım: python3 tools/plot_log.py <girdi.csv> <cikti.png>")
    main(sys.argv[1], sys.argv[2])
