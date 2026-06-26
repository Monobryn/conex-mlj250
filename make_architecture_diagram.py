"""
Generate the architecture diagram for the Beam Stabilizer GUI.

Renders a component + signal-flow diagram of mlj250_conex_gui.py and its
dependencies, saved as architecture.png and architecture.svg next to this
script. Uses only matplotlib (already a project dependency) so it is fully
reproducible with no extra tooling:

    python3 make_architecture_diagram.py
"""

import os
import matplotlib
matplotlib.use("Agg")            # headless: write files, no window
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# ── Palette ───────────────────────────────────────────────────────────────────
C_HW    = "#37474f"   # external hardware
C_DRV   = "#00695c"   # drivers / serial layer
C_WORK  = "#1565c0"   # worker QThreads
C_GUI   = "#4527a0"   # GUI thread
C_SUB   = "#5e35b1"   # widgets inside the GUI
C_FILE  = "#6d4c41"   # persisted files
EDGE_SIG = "#2e7d32"  # signals (worker -> GUI)
EDGE_CMD = "#c62828"  # commands (GUI -> worker)
EDGE_IO  = "#455a64"  # hardware / file I/O
TEXT_LT  = "white"


def box(ax, x, y, w, h, title, subtitle="", color=C_GUI, fontsize=11,
        text_color=TEXT_LT, alpha=1.0):
    """Draw a rounded box centred at (x, y) with a title and optional subtitle."""
    patch = FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.02,rounding_size=0.12",
        linewidth=1.2, edgecolor="white", facecolor=color, alpha=alpha,
        zorder=2,
    )
    ax.add_patch(patch)
    if subtitle:
        ax.text(x, y + h * 0.16, title, ha="center", va="center",
                color=text_color, fontsize=fontsize, fontweight="bold", zorder=3)
        ax.text(x, y - h * 0.22, subtitle, ha="center", va="center",
                color=text_color, fontsize=fontsize - 2.5, zorder=3)
    else:
        ax.text(x, y, title, ha="center", va="center",
                color=text_color, fontsize=fontsize, fontweight="bold", zorder=3)


def arrow(ax, p0, p1, color, label="", rad=0.0, label_pos=0.5,
          label_dy=0.18, fontsize=8.0, style="-|>"):
    """Draw a curved arrow from p0 to p1 with an optional mid-point label."""
    a = FancyArrowPatch(
        p0, p1, arrowstyle=style, mutation_scale=14,
        connectionstyle=f"arc3,rad={rad}", linewidth=1.6,
        color=color, zorder=1,
    )
    ax.add_patch(a)
    if label:
        mx = p0[0] + (p1[0] - p0[0]) * label_pos
        my = p0[1] + (p1[1] - p0[1]) * label_pos + label_dy
        ax.text(mx, my, label, ha="center", va="center", color=color,
                fontsize=fontsize, fontweight="bold", zorder=4,
                bbox=dict(boxstyle="round,pad=0.18", fc="white",
                          ec=color, lw=0.8, alpha=0.95))


def main():
    fig, ax = plt.subplots(figsize=(15, 9))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis("off")
    ax.set_title("Beam Stabilizer GUI — Architecture (components & signal flow)",
                 fontsize=15, fontweight="bold", pad=14)

    # Column header band labels
    for cx, lbl in [(2.0, "External hardware"), (5.4, "Driver layer"),
                    (9.0, "Worker threads"), (13.2, "GUI thread (main)")]:
        ax.text(cx, 9.35, lbl, ha="center", va="center", fontsize=10,
                fontstyle="italic", color="#555555")

    # ── Hardware column ────────────────────────────────────────────────────────
    box(ax, 2.0, 7.4, 3.0, 1.2, "Thorlabs MLJ250", "motorized lab jack (USB)",
        color=C_HW)
    box(ax, 2.0, 4.0, 3.0, 1.2, "Newport CONEX-PSD9", "beam position sensor (USB)",
        color=C_HW)

    # ── Driver column ──────────────────────────────────────────────────────────
    box(ax, 5.4, 7.4, 3.0, 1.2, "Thorlabs Kinesis", ".NET DLLs via pythonnet/clr",
        color=C_DRV)
    box(ax, 5.4, 4.0, 3.0, 1.2, "conex_psd9.ConexPSD", "pyserial 921600 8N1",
        color=C_DRV)

    # ── Worker thread column ───────────────────────────────────────────────────
    box(ax, 9.0, 7.4, 3.0, 1.4, "StageWorker", "QThread • command queue\n+ feedback law",
        color=C_WORK)
    box(ax, 9.0, 4.0, 3.0, 1.4, "PSDWorker", "QThread • 10 Hz poll\n+ 14.4 s rolling avg",
        color=C_WORK)

    # ── GUI column (MainWindow contains the two plot widgets) ───────────────────
    box(ax, 13.2, 5.7, 3.6, 5.0, "", "", color=C_GUI, alpha=0.30)
    ax.text(13.2, 7.85, "MainWindow", ha="center", va="center", color=TEXT_LT,
            fontsize=12, fontweight="bold", zorder=3)
    ax.text(13.2, 7.45, "QMainWindow", ha="center", va="center", color=TEXT_LT,
            fontsize=8.5, zorder=3)
    box(ax, 13.2, 6.4, 3.0, 0.9, "BeamPlotCanvas", "live X/Y + setpoint",
        color=C_SUB, fontsize=10)
    box(ax, 13.2, 5.2, 3.0, 0.9, "FeedbackPlotWindow", "live time-series",
        color=C_SUB, fontsize=10)

    # ── Persistence row ────────────────────────────────────────────────────────
    box(ax, 11.3, 1.4, 2.7, 1.0, "stage_state.json", "ref / last pos / homed",
        color=C_FILE, fontsize=9.5)
    box(ax, 14.6, 1.4, 2.7, 1.0, "feedback_trace_*.csv", "saved on disable",
        color=C_FILE, fontsize=9.5)

    # ── Hardware <-> driver I/O ────────────────────────────────────────────────
    arrow(ax, (3.5, 7.4), (3.9, 7.4), EDGE_IO, style="<|-|>")
    arrow(ax, (3.5, 4.0), (3.9, 4.0), EDGE_IO, style="<|-|>")
    ax.text(3.7, 7.75, "USB", ha="center", fontsize=7.5, color=EDGE_IO)
    ax.text(3.7, 4.35, "USB", ha="center", fontsize=7.5, color=EDGE_IO)

    # ── Driver <-> worker ──────────────────────────────────────────────────────
    arrow(ax, (6.9, 7.4), (7.5, 7.4), EDGE_IO, style="<|-|>")
    arrow(ax, (6.9, 4.0), (7.5, 4.0), EDGE_IO, style="<|-|>")
    ax.text(7.2, 7.75, ".NET calls", ha="center", fontsize=7.5, color=EDGE_IO)
    ax.text(7.2, 4.35, "get_position()", ha="center", fontsize=7.5, color=EDGE_IO)

    # ── StageWorker <-> MainWindow ─────────────────────────────────────────────
    # GUI -> worker: command queue (red)
    arrow(ax, (11.4, 7.9), (10.5, 7.9), EDGE_CMD, rad=0.0,
          label="cmd_* queue + update_psd()", label_pos=0.5, label_dy=0.28,
          fontsize=7.6)
    # worker -> GUI: signals (green)
    arrow(ax, (10.5, 7.0), (11.4, 7.0), EDGE_SIG, rad=0.0,
          label="signals: connected / position_updated /\nhomed / move_done / feedback_status",
          label_pos=0.5, label_dy=-0.42, fontsize=7.2)

    # ── PSDWorker <-> MainWindow ───────────────────────────────────────────────
    arrow(ax, (10.5, 4.4), (11.4, 4.4), EDGE_SIG, rad=0.0,
          label="signals: position_updated /\naveraged_position(mean, count)",
          label_pos=0.5, label_dy=0.40, fontsize=7.2)
    arrow(ax, (11.4, 3.5), (10.5, 3.5), EDGE_CMD, rad=0.0,
          label="start / stop", label_pos=0.5, label_dy=-0.30, fontsize=7.6)

    # ── Averaged beam -> feedback (GUI re-dispatch from PSDWorker to StageWorker)
    arrow(ax, (9.0, 4.7), (9.0, 6.7), EDGE_CMD, rad=0.0,
          label="_on_psd_averaged →\nupdate_psd()", label_pos=0.5,
          label_dy=0.0, fontsize=7.2)

    # ── Persistence I/O ────────────────────────────────────────────────────────
    arrow(ax, (12.6, 3.2), (11.7, 1.95), EDGE_IO, style="<|-|>",
          label="load / save", label_pos=0.5, label_dy=0.25, fontsize=7.2)
    arrow(ax, (13.6, 4.7), (14.4, 1.95), EDGE_IO, style="-|>",
          label="write", label_pos=0.55, label_dy=0.20, fontsize=7.2)

    # ── Legend ─────────────────────────────────────────────────────────────────
    ax.add_patch(FancyArrowPatch((0.4, 0.7), (1.4, 0.7), arrowstyle="-|>",
                 mutation_scale=12, color=EDGE_SIG, linewidth=1.6))
    ax.text(1.55, 0.7, "pyqtSignal (worker → GUI)", va="center", fontsize=8,
            color=EDGE_SIG)
    ax.add_patch(FancyArrowPatch((5.4, 0.7), (6.4, 0.7), arrowstyle="-|>",
                 mutation_scale=12, color=EDGE_CMD, linewidth=1.6))
    ax.text(6.55, 0.7, "command / call (GUI → worker)", va="center", fontsize=8,
            color=EDGE_CMD)
    ax.add_patch(FancyArrowPatch((11.0, 0.7), (12.0, 0.7), arrowstyle="<|-|>",
                 mutation_scale=12, color=EDGE_IO, linewidth=1.6))
    ax.text(12.15, 0.7, "hardware / file I/O", va="center", fontsize=8,
            color=EDGE_IO)

    fig.tight_layout()
    here = os.path.dirname(os.path.abspath(__file__))
    png = os.path.join(here, "architecture.png")
    svg = os.path.join(here, "architecture.svg")
    fig.savefig(png, dpi=150, bbox_inches="tight", facecolor="white")
    fig.savefig(svg, bbox_inches="tight", facecolor="white")
    print(f"Wrote {png}")
    print(f"Wrote {svg}")


if __name__ == "__main__":
    main()
