#!/usr/bin/env python3
"""Genera scans .pcd sinteticos con el mismo formato del Newer College.

Sirve para verificar que el pipeline compila y corre de extremo a extremo antes
de bajar el dataset real, que pesa decenas de gigabytes.

La escena es una habitacion rectangular con un par de columnas. Un sensor
giratorio recorre una trayectoria recta por el centro y en cada posicion lanza
rayos que se intersecan con las paredes. El resultado es geometria consistente
entre scans, que es lo que el ICP necesita para converger: nubes aleatorias no
sirven para probar el pipeline completo.

Los archivos replican el formato de un Ouster: 9 campos de tamanos mixtos
(x y z intensity t reflectivity ring ambient range), para ejercitar el parseo
de offsets del lector.

Uso:
    python3 tools/make_synthetic_scans.py --out data/synthetic --scans 20
    python3 tools/make_synthetic_scans.py --out data/synthetic --ascii
"""

import argparse
import math
import os
import struct

# Campos identicos a los de un Ouster OS1, en el mismo orden.
FIELDS = [
    ("x",            4, "F"),
    ("y",            4, "F"),
    ("z",            4, "F"),
    ("intensity",    4, "F"),
    ("t",            4, "U"),
    ("reflectivity", 2, "U"),
    ("ring",         1, "U"),
    ("ambient",      2, "U"),
    ("range",        4, "U"),
]

# Habitacion: 20 m x 12 m x 4 m, centrada en el origen en X e Y.
ROOM_X, ROOM_Y, ROOM_Z = 10.0, 6.0, 4.0
COLUMNS = [(3.0, 2.0, 0.4), (-4.0, -1.5, 0.4)]  # (cx, cy, radio)


def ray_hit(ox, oy, oz, dx, dy, dz, max_range):
    """Distancia al primer obstaculo a lo largo del rayo, o None.

    Marcha de rayos con paso fijo. No es eficiente, pero para generar datos de
    prueba da igual y el codigo queda corto y legible.
    """
    step = 0.05
    t = 0.3
    while t < max_range:
        px, py, pz = ox + dx * t, oy + dy * t, oz + dz * t
        if abs(px) >= ROOM_X or abs(py) >= ROOM_Y:
            return t
        if pz <= 0.0 or pz >= ROOM_Z:
            return t
        for cx, cy, r in COLUMNS:
            if (px - cx) ** 2 + (py - cy) ** 2 <= r * r:
                return t
        t += step
    return None


def make_scan(sensor_x, n_azimuth=360, n_ring=32, max_range=40.0):
    """Devuelve una lista de (x, y, z, intensity, ring, rng_mm) en el marco del sensor."""
    ox, oy, oz = sensor_x, 0.0, 1.5
    points = []
    for ri in range(n_ring):
        # Elevacion de -20 a +20 grados, como un OS1.
        elev = math.radians(-20.0 + 40.0 * ri / max(n_ring - 1, 1))
        for ai in range(n_azimuth):
            azim = 2.0 * math.pi * ai / n_azimuth
            dx = math.cos(elev) * math.cos(azim)
            dy = math.cos(elev) * math.sin(azim)
            dz = math.sin(elev)
            t = ray_hit(ox, oy, oz, dx, dy, dz, max_range)
            if t is None:
                # Retorno invalido: se emite como NaN, igual que el sensor real.
                # Asi se prueba que el lector los descarta.
                points.append((float("nan"), float("nan"), float("nan"), 0.0, ri, 0))
                continue
            points.append((dx * t, dy * t, dz * t, 100.0, ri, int(t * 1000)))
    return points


def write_pcd(path, points, ascii_mode):
    names = " ".join(f[0] for f in FIELDS)
    sizes = " ".join(str(f[1]) for f in FIELDS)
    types = " ".join(f[2] for f in FIELDS)
    counts = " ".join("1" for _ in FIELDS)
    n = len(points)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {names}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {'ascii' if ascii_mode else 'binary'}\n"
    )

    if ascii_mode:
        with open(path, "w") as fh:
            fh.write(header)
            for i, (x, y, z, inten, ring, rng) in enumerate(points):
                fh.write(f"{x} {y} {z} {inten} {i} 0 {ring} 0 {rng}\n")
    else:
        # '<' fuerza little-endian y sin padding, que es como PCL escribe binary.
        fmt = "<ffffIHBHI"
        with open(path, "wb") as fh:
            fh.write(header.encode("ascii"))
            for i, (x, y, z, inten, ring, rng) in enumerate(points):
                fh.write(struct.pack(fmt, x, y, z, inten, i, 0, ring, 0, rng))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="Directorio de salida")
    ap.add_argument("--scans", type=int, default=20, help="Numero de scans (20)")
    ap.add_argument("--ascii", action="store_true", help="Escribir DATA ascii en vez de binary")
    ap.add_argument("--azimuth", type=int, default=360, help="Rayos por anillo (360)")
    ap.add_argument("--rings", type=int, default=32, help="Anillos verticales (32)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # Trayectoria: el sensor avanza 0.3 m por scan a lo largo de X.
    # El movimiento es indispensable: con el sensor quieto, KISS-ICP detecta que
    # no hay desplazamiento y el mapa no crece.
    base_sec = 1583836591
    for i in range(args.scans):
        sensor_x = -6.0 + 0.3 * i
        points = make_scan(sensor_x, args.azimuth, args.rings)

        # Timestamp a 10 Hz. Se formatea SIN rellenar a 9 digitos a proposito,
        # para que el ordenamiento numerico del programa quede probado.
        nsec = (i * 100_000_000) % 1_000_000_000
        sec = base_sec + (i * 100_000_000) // 1_000_000_000
        name = f"cloud_{sec}_{nsec}.pcd"

        write_pcd(os.path.join(args.out, name), points, args.ascii)
        print(f"  {name}  ({len(points)} puntos)")

    print(f"\n{args.scans} scans en {args.out}")
    print("Pruebe con:")
    print(f"  ./build/recon --scans {args.out} --out test.ply "
          f"--tsdf-voxel 0.1 --icp-voxel 0.5 --min-range 0.5")


if __name__ == "__main__":
    main()
