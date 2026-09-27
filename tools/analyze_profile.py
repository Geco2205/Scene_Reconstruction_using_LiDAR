#!/usr/bin/env python3
"""analyze_profile.py -- estadisticas y graficos del perfilado por etapa.

Lee cada subcarpeta de <results> con un timings.csv (una por maquina, creada
por tools/run_profile.sh) y escribe en <results>/analysis/:

  summary.md     tablas por maquina y comparacion entre maquinas
  summary.csv    las mismas estadisticas en formato tabular
  fig_breakdown.png   tiempo medio por scan desglosado por etapa, por maquina
  fig_stages_box.png  distribucion de cada etapa por maquina (escala log)
  fig_timeline_<maquina>.png  tiempo por scan y memoria a lo largo de la secuencia

Por etapa: n, media, desviacion estandar, IC 95 % de la media, min, p50, p95,
max, porcentaje del tiempo por scan y razon CPU/pared.

Uso:
  python3 tools/analyze_profile.py results [--skip-first N]

Periodo: con --pipeline las etapas se traslapan y la suma de etapas deja de ser
el tiempo por scan. Si timings.csv trae la columna done_ms (instante en que cada
scan termino), el periodo entre scans consecutivos es la medida real de
rendimiento y de ahi sale el throughput. Sin done_ms (CSV viejos) se usa la suma.

--skip-first descarta los primeros N scans de cada maquina. Los primeros
registros de KISS-ICP son mas baratos porque el mapa local aun esta vacio. Por
defecto no descarta nada.

Dependencias: numpy y matplotlib.
"""

import argparse
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

STAGES = ["read", "filter", "convert", "register", "transform", "integrate"]

# Paleta categorica, mismo orden en todas las figuras.
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
           "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"


def t_critical_95(dof):
    """Valor t de dos colas al 95 %. Con n > 100 ya es practicamente 1.96."""
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 10: 2.228,
             20: 2.086, 30: 2.042, 60: 2.000, 120: 1.980}
    if dof <= 0:
        return float("nan")
    for k in sorted(table):
        if dof <= k:
            return table[k]
    return 1.960


def stats(values):
    v = np.asarray(values, dtype=float)
    n = v.size
    if n == 0:
        return None
    mean = float(v.mean())
    std = float(v.std(ddof=1)) if n > 1 else 0.0
    half = t_critical_95(n - 1) * std / math.sqrt(n) if n > 1 else float("nan")
    return {
        "n": n, "mean": mean, "std": std, "ci95": half,
        "min": float(v.min()), "p50": float(np.percentile(v, 50)),
        "p95": float(np.percentile(v, 95)), "max": float(v.max()),
    }


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def read_system(path):
    info = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                info[k.strip()] = v.strip()
    return info


def peak_rss_from_log(path):
    """RSS pico reportado por /usr/bin/time -v, en KiB (None si no esta)."""
    if not path.exists():
        return None
    m = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", path.read_text())
    return int(m.group(1)) if m else None


def load_machine(folder, skip_first):
    timings = read_csv(folder / "timings.csv")
    if not timings:
        return None
    if skip_first:
        timings = {k: v[skip_first:] for k, v in timings.items()}
    mesh = read_csv(folder / "mesh.csv") if (folder / "mesh.csv").exists() else {}
    total = sum(timings[f"{s}_ms"] for s in STAGES)
    # Periodo entre scans terminados. El primero se mide desde el inicio del
    # ciclo (done_ms del primero). Con --skip-first el primer periodo se pierde.
    period = None
    if "done_ms" in timings and timings["done_ms"].size > 1:
        done = timings["done_ms"]
        period = np.diff(done) if skip_first else np.diff(np.concatenate(([0.0], done)))
    return {
        "label": folder.name,
        "system": read_system(folder / "system.txt"),
        "timings": timings,
        "total": total,
        "period": period,
        "mesh": mesh,
        "peak_rss_kb": peak_rss_from_log(folder / "run.log"),
    }


def scan_period_ms(m):
    """Tiempo medio entre scans: periodo real si hay done_ms, si no la suma."""
    return float(m["period"].mean()) if m["period"] is not None else float(m["total"].mean())


def variant(m):
    """Descripcion corta de la build y el modo de ejecucion."""
    s = m["system"]
    opts = [k for k in ("OPT_NATIVE", "OPT_LTO", "OPT_SOA_ALIGNED") if s.get(k, "").upper() == "ON"]
    neon = s.get("OPT_NEON", "")
    txt = ", ".join(o.replace("OPT_", "").lower() for o in opts) or "base"
    if neon.upper() == "OFF":
        txt += ", no-neon"
    return f"{txt}; pipeline {s.get('pipeline', 'off')}"


def fmt(x, digits=2):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "–"
    return f"{x:.{digits}f}"


def machine_rows(m):
    """Filas de estadisticas por etapa para una maquina."""
    t = m["timings"]
    total_mean = float(m["total"].mean())
    rows = []
    for s in STAGES:
        st = stats(t[f"{s}_ms"])
        cpu = t.get(f"{s}_cpu_ms")
        ratio = float(cpu.sum() / t[f"{s}_ms"].sum()) if cpu is not None and t[f"{s}_ms"].sum() > 0 else float("nan")
        rows.append({"stage": s, **st, "share": 100 * st["mean"] / total_mean, "cpu_ratio": ratio})
    st = stats(m["total"])
    rows.append({"stage": "total/scan", **st, "share": 100.0, "cpu_ratio": float("nan")})
    if m["period"] is not None:
        st = stats(m["period"])
        rows.append({"stage": "period", **st, "share": float("nan"), "cpu_ratio": float("nan")})
    if m["mesh"]:
        st = stats(m["mesh"]["mesh_ms"])
        ratio = float(m["mesh"]["mesh_cpu_ms"].sum() / m["mesh"]["mesh_ms"].sum())
        rows.append({"stage": "mesh (once)", **st, "share": float("nan"), "cpu_ratio": ratio})
    return rows


def write_summary(machines, out_dir, skip_first):
    md = ["# Profiling summary", ""]
    md.append("Generated by `tools/analyze_profile.py`. Times in ms (wall clock). "
              "CI95 is the half-width of the 95 % confidence interval of the mean. "
              "CPU/wall is process CPU time divided by wall time during the stage "
              "(≈1 serial, >1 multithreaded). `mesh` runs once per sequence and is "
              "repeated on the final volume to collect samples. "
              "`total/scan` is the sum of the stages (work per scan); `period` is the "
              "measured time between consecutive finished scans. They match in "
              "sequential mode and differ with `--pipeline`, where stages overlap: "
              "throughput comes from `period`. With `--pipeline` the CPU columns use "
              "per-thread CPU time and do not include KISS-ICP's TBB workers.")
    if skip_first:
        md.append(f"The first {skip_first} scans of each machine were discarded as warm-up.")
    md.append("")

    csv_rows = []
    header = "| Stage | n | Mean | Std | CI95 | p50 | p95 | Max | Share % | CPU/wall |"
    sep = "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
    for m in machines:
        sysinfo = m["system"]
        md.append(f"## {m['label']}")
        md.append("")
        md.append(f"- CPU: {sysinfo.get('cpu_model', '?')} ({sysinfo.get('cpu_arch', '?')}, "
                  f"{sysinfo.get('cpu_count', '?')} threads), RAM {sysinfo.get('mem_total', '?')}")
        md.append(f"- OS: {sysinfo.get('os', '?')}; compiler: {sysinfo.get('compiler', '?')}")
        md.append(f"- Commit: {sysinfo.get('git_commit', '?')}, build: {sysinfo.get('build_type', '?')}, "
                  f"args: `{sysinfo.get('extra_args', '')}`")
        md.append(f"- Variant: {variant(m)}")
        rss = m["timings"]["rss_kb"]
        peak = m["peak_rss_kb"]
        md.append(f"- Memory: RSS after first scan {rss[0] / 1024:.0f} MiB, after last scan "
                  f"{rss[-1] / 1024:.0f} MiB" + (f", process peak {peak / 1024:.0f} MiB" if peak else ""))
        md.append(f"- Throughput: {1000 / scan_period_ms(m):.2f} scans/s "
                  f"(sensor runs at 10 scans/s)")
        md.append("")
        md.append(header)
        md.append(sep)
        for r in machine_rows(m):
            md.append(f"| {r['stage']} | {r['n']} | {fmt(r['mean'])} | {fmt(r['std'])} | {fmt(r['ci95'])} | "
                      f"{fmt(r['p50'])} | {fmt(r['p95'])} | {fmt(r['max'])} | {fmt(r['share'], 1)} | "
                      f"{fmt(r['cpu_ratio'])} |")
            csv_rows.append({"machine": m["label"], **r})
        md.append("")

    if len(machines) > 1:
        md.append("## Comparison (mean ms per scan)")
        md.append("")
        md.append("| Stage | " + " | ".join(m["label"] for m in machines) + " |")
        md.append("|---|" + "---:|" * len(machines))
        per = {m["label"]: {r["stage"]: r for r in machine_rows(m)} for m in machines}
        for s in STAGES + ["total/scan", "period", "mesh (once)"]:
            cells = [fmt(per[m["label"]].get(s, {}).get("mean")) for m in machines]
            md.append(f"| {s} | " + " | ".join(cells) + " |")
        cells = [fmt(1000 / scan_period_ms(m)) for m in machines]
        md.append("| **throughput (scans/s)** | " + " | ".join(cells) + " |")
        base = machines[0]
        cells = [fmt(scan_period_ms(base) / scan_period_ms(m)) for m in machines]
        md.append(f"| speedup vs {base['label']} | " + " | ".join(cells) + " |")
        md.append("| variant | " + " | ".join(variant(m) for m in machines) + " |")
        md.append("")

    (out_dir / "summary.md").write_text("\n".join(md) + "\n")
    keys = ["machine", "stage", "n", "mean", "std", "ci95", "min", "p50", "p95", "max", "share", "cpu_ratio"]
    with open(out_dir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in csv_rows:
            w.writerow({k: r[k] for k in keys})


# --- Figuras -------------------------------------------------------------------

def style_axes(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(TEXT_2)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=TEXT_2, labelsize=8)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def fig_breakdown(machines, out_dir, plt):
    """Barra horizontal apilada: tiempo medio por scan, segmentado por etapa."""
    fig, ax = plt.subplots(figsize=(7.0, 0.55 * len(machines) + 1.4))
    style_axes(ax)
    ax.grid(axis="y", visible=False)
    labels = [m["label"] for m in machines]
    y = np.arange(len(machines))[::-1]
    left = np.zeros(len(machines))
    for i, s in enumerate(STAGES):
        vals = np.array([m["timings"][f"{s}_ms"].mean() for m in machines])
        ax.barh(y, vals, left=left, height=0.55, color=PALETTE[i], label=s,
                edgecolor="white", linewidth=1.0)
        left += vals
    for yi, total, m in zip(y, left, machines):
        per = scan_period_ms(m)
        extra = f" (period {per:.1f})" if m["period"] is not None and abs(per - total) > 0.05 * total else ""
        ax.text(total, yi, f"  {total:.1f} ms{extra}", va="center", fontsize=8, color=TEXT)
    ax.set_yticks(y, labels, fontsize=8, color=TEXT)
    ax.set_xlabel("Mean time per scan (ms)", fontsize=8, color=TEXT_2)
    ax.set_xlim(0, left.max() * 1.35)
    ax.legend(ncol=len(STAGES), fontsize=7, frameon=False, loc="lower left",
              bbox_to_anchor=(0, 1.0), handlelength=1.2, columnspacing=1.0)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_breakdown.png", dpi=300)
    plt.close(fig)


def fig_stages_box(machines, out_dir, plt):
    """Distribucion de cada etapa (y la malla) por maquina, escala logaritmica."""
    groups = STAGES + (["mesh"] if all(m["mesh"] for m in machines) else [])
    k = len(machines)
    width = 0.8 / k
    fig, ax = plt.subplots(figsize=(7.0, 3.0))
    style_axes(ax)
    ax.grid(axis="x", visible=False)
    for j, m in enumerate(machines):
        data = [m["timings"][f"{s}_ms"] if s != "mesh" else m["mesh"]["mesh_ms"] for s in groups]
        pos = np.arange(len(groups)) - 0.4 + width * (j + 0.5)
        color = PALETTE[j]
        ax.boxplot(data, positions=pos, widths=width * 0.8, whis=(5, 95), showfliers=False,
                   patch_artist=True,
                   boxprops=dict(facecolor=color, edgecolor=color, alpha=0.35, linewidth=1),
                   medianprops=dict(color=color, linewidth=2),
                   whiskerprops=dict(color=color, linewidth=1),
                   capprops=dict(color=color, linewidth=1))
        ax.plot([], [], color=color, linewidth=6, alpha=0.6, label=m["label"])
    ax.set_yscale("log")
    ax.set_xticks(np.arange(len(groups)), groups, fontsize=8, color=TEXT)
    ax.set_ylabel("Time (ms, log scale)", fontsize=8, color=TEXT_2)
    ax.set_title("Per-stage distribution (box: p25–p75, whiskers: p5–p95)",
                 fontsize=8, color=TEXT_2, loc="left")
    if k > 1:
        ax.legend(fontsize=7, frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(out_dir / "fig_stages_box.png", dpi=300)
    plt.close(fig)


def fig_timeline(m, out_dir, plt):
    """Tiempo por scan (arriba) y memoria residente (abajo) a lo largo de la secuencia."""
    t = m["timings"]
    x = t["scan"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.0, 3.8), sharex=True,
                                   gridspec_kw=dict(height_ratios=[2, 1]))
    for ax in (ax1, ax2):
        style_axes(ax)
    series = [("total", m["total"], TEXT_2), ("register", t["register_ms"], PALETTE[3]),
              ("integrate", t["integrate_ms"], PALETTE[5])]
    for name, y, color in series:
        ax1.plot(x, y, color=color, linewidth=1.2, label=name)
    ax1.set_ylabel("Time (ms)", fontsize=8, color=TEXT_2)
    ax1.set_ylim(bottom=0)
    ax1.legend(fontsize=7, frameon=False, ncol=3, loc="upper left")
    ax1.set_title(f"{m['label']}: time per scan and resident memory", fontsize=8,
                  color=TEXT_2, loc="left")
    ax2.plot(x, t["rss_kb"] / 1024, color=PALETTE[0], linewidth=1.2)
    ax2.set_ylabel("RSS (MiB)", fontsize=8, color=TEXT_2)
    ax2.set_xlabel("Scan index", fontsize=8, color=TEXT_2)
    ax2.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(out_dir / f"fig_timeline_{m['label']}.png", dpi=300)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("results", type=Path, help="Carpeta con una subcarpeta por maquina")
    ap.add_argument("--skip-first", type=int, default=0, help="Scans de calentamiento a descartar")
    ap.add_argument("--no-plots", action="store_true", help="Solo tablas, sin figuras")
    ap.add_argument("--ref", default=None,
                    help="Carpeta de referencia para el speedup (por defecto 'base' si existe)")
    args = ap.parse_args()

    folders = sorted(p for p in args.results.iterdir() if (p / "timings.csv").exists())
    machines = [m for m in (load_machine(p, args.skip_first) for p in folders) if m]
    # La referencia del speedup es la primera columna: "base" si existe
    # (resultados de run_optimizations.sh), si no --ref, si no la primera.
    ref = args.ref or "base"
    machines.sort(key=lambda m: m["label"] != ref)
    if not machines:
        sys.exit(f"No hay timings.csv en ninguna subcarpeta de {args.results}")
    if len(machines) > len(PALETTE):
        sys.exit(f"Maximo {len(PALETTE)} maquinas por analisis")

    out_dir = args.results / "analysis"
    out_dir.mkdir(exist_ok=True)
    write_summary(machines, out_dir, args.skip_first)

    if not args.no_plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.family"] = "DejaVu Sans"
        fig_breakdown(machines, out_dir, plt)
        fig_stages_box(machines, out_dir, plt)
        for m in machines:
            fig_timeline(m, out_dir, plt)

    # Con menos de 100 muestras el intervalo de confianza de la media no sirve.
    # Se valida etapa por etapa, incluida la malla.
    MIN_SAMPLES = 100
    problems = 0
    for m in machines:
        n = len(m["total"])
        warn = "" if n > MIN_SAMPLES else "  <-- menos de 100 muestras"
        print(f"{m['label']:>16}: {n} scans, {m['total'].mean():.1f} ms trabajo/scan, "
              f"{scan_period_ms(m):.1f} ms periodo{warn}")
        if n <= MIN_SAMPLES:
            problems += 1
        for s in STAGES:
            k = int(m["timings"][f"{s}_ms"].size)
            if k <= MIN_SAMPLES:
                print(f"{'':>18}etapa {s}: {k} muestras  <-- FAIL (se necesitan >100)")
                problems += 1
        if m["mesh"]:
            k = int(m["mesh"]["mesh_ms"].size)
            if k <= MIN_SAMPLES:
                print(f"{'':>18}etapa mesh: {k} muestras  <-- FAIL (se necesitan >100; "
                      f"repetir con --mesh-repeats 101)")
                problems += 1
        else:
            print(f"{'':>18}etapa mesh: sin mesh.csv  <-- FAIL (falta --mesh-csv)")
            problems += 1
    if problems:
        print(f"\n{problems} etapa(s) con {MIN_SAMPLES} muestras o menos. Se requieren "
              f"mas de {MIN_SAMPLES} por etapa; estos datos no son publicables.")
    print(f"Resultados en {out_dir}/")


if __name__ == "__main__":
    main()
