# Scene Reconstruction using LiDAR

CPU prototype of a LiDAR scene reconstruction pipeline, developed for the course
EL5859 Heterogeneous Computing (Instituto Tecnológico de Costa Rica, 2026).
It turns a sequence of LiDAR scans into a 3D triangle mesh of the environment.

```
.pcd scans → range filter (NEON/scalar) → KISS-ICP (pose) → VDBFusion (TSDF) → .ply mesh
```

This is the CPU baseline of the project. Later stages will offload the bottlenecks
found here to a GPU (NVIDIA Jetson Nano, CUDA) and an FPGA (AMD Kria KV260, HLS).

## What is reused and what we wrote

| Component | Origin | License |
|---|---|---|
| 3D registration (LiDAR odometry) | [KISS-ICP](https://github.com/PRBonn/kiss-icp) `v1.0.0`, used as a library | MIT |
| TSDF integration and mesh extraction | [VDBFusion](https://github.com/PRBonn/vdbfusion) `v0.1.6`, used as a library | MIT |
| Pipeline (`src/main.cpp`) | Written for this project, based on VDBFusion's `examples/cpp/kitti_pipeline.cpp` | — |
| PCD reader (`src/PcdReader.cpp`) | Written for this project, following the [PCD file format specification](https://pointclouds.org/documentation/tutorials/pcd_file_format.html) | — |
| Range filter with ARM NEON (`src/RangeFilter.cpp`) | Written for this project | — |
| Scan ordering, PLY writer, timer (`src/Io.cpp`) | Written for this project | — |
| Synthetic scan generator (`tools/make_synthetic_scans.py`) | Written for this project | — |
| Per-stage profiler (`src/Profiler.cpp`) and profiling scripts (`tools/run_profile.sh`, `tools/analyze_profile.py`) | Written for this project | — |




## Dependencies

Fedora:

```bash
sudo dnf install gcc-c++ cmake git eigen3-devel tbb-devel openvdb-devel \
                 imath-devel boost-devel blosc-devel python3-numpy
```

Ubuntu (including the Kria):

```bash
sudo apt install build-essential cmake git libeigen3-dev libtbb-dev \
                 libopenvdb-dev libboost-iostreams-dev libblosc-dev python3-numpy
```

OpenVDB must come from the system package manager. If CMake cannot find it, the
configuration stops with an error instead of trying to build it from source.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The first configuration needs internet access to download KISS-ICP and VDBFusion.

## Quick test (no dataset needed)

```bash
python3 tools/make_synthetic_scans.py --out data/synthetic --scans 20
./build/recon --scans data/synthetic --out synthetic.ply
```

Expected: the program prints the mesh size and the average time per stage, and
writes `synthetic.ply`. This only checks that everything builds and runs; the
synthetic scene is too simple to evaluate registration accuracy.

## Data

We use the [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/)
(2020, Ouster OS1-64 at 10 Hz), sequence 01 "short experiment".

### Option A: sample (300 scans, ~30 s of walking)

A ready-to-use sample with the matching ground-truth poses is attached to the
[v0.1.0 release](https://github.com/Geco2205/Scene_Reconstruction_using_LiDAR/releases/tag/v0.1.0):

```bash
wget https://github.com/Geco2205/Scene_Reconstruction_using_LiDAR/releases/download/v0.1.0/ncd_sample.zip
unzip ncd_sample.zip -d data
./build/recon --scans data/ncd_sample/scans --out sample.ply --icp-voxel 1.0
```

### Option B: full dataset

1. Request access through the form on the dataset website.
2. In `2020-ouster-os1-64-realsense`, open the folder of sequence 01 (short
   experiment) and download `raw_format/ouster_zip_files/` (10 zips)
   and `ground_truth/`.
3. Unzip all 10 files into the same folder. The scans are spread across the zips
   without temporal order, so all of them are needed to get a continuous sequence:

```bash
mkdir -p data/ncd
for z in ouster_scan-*.zip; do unzip -jq "$z" -d data/ncd; done
rm -f data/ncd/*"(1)".pcd    # one file is duplicated in the zips
ls data/ncd | wc -l           # expected: 15301
```

## Usage

```bash
./build/recon --scans <dir> [options]
```

| Option | Default | Description |
|---|---|---|
| `--scans <dir>` | required | Directory with the `.pcd` scans |
| `--out <file>` | `mesh.ply` | Output mesh |
| `--max-scans <n>` | `0` (all) | Number of scans to process |
| `--tsdf-voxel <m>` | `0.10` | TSDF voxel size |
| `--icp-voxel <m>` | `0.50` | KISS-ICP voxel size |
| `--min-range <m>` | `1.0` | Minimum point range |
| `--max-range <m>` | `60.0` | Maximum point range |
| `--timing-csv <file>` | — | Write per-scan timings and memory to a CSV file |
| `--mesh-repeats <n>` | `1` | Repeat mesh extraction `n` times on the final volume |
| `--mesh-csv <file>` | — | Write the time of each mesh extraction to a CSV file |

Scans are processed in timestamp order. File names follow
`cloud_<sec>_<nsec>.pcd` and are sorted numerically, because the nanosecond
field does not always have 9 digits.

## Outputs

- **`<out>.ply`**: triangle mesh (ASCII PLY). Open it with MeshLab or CloudCompare.
- **`<timing-csv>`**: one row per scan. For each stage (`read`, `filter`,
  `convert`, `register`, `transform`, `integrate`) there is a wall-clock column
  `<stage>_ms` and a process CPU-time column `<stage>_cpu_ms`, plus
  `scan, points_in, points_kept` and `rss_kb` (resident memory after the scan).
- **`<mesh-csv>`**: one row per mesh extraction with
  `repeat, mesh_ms, mesh_cpu_ms, vertices, triangles`.

## Profiling

Each stage is measured per scan with two clocks: wall time (`steady_clock`) and
process CPU time (`CLOCK_PROCESS_CPUTIME_ID`, summed over all threads). Their
ratio tells how many cores a stage used on average: about 1.0 means serial,
above 1.0 means the stage is multithreaded (KISS-ICP uses TBB internally).
Memory is read from `/proc/self/statm` after every scan, and the process peak
from `getrusage`/`/usr/bin/time -v`.

The 300-scan sample gives 300 samples for every per-scan stage. Mesh extraction
happens once per sequence, so it is repeated on the final volume
(`--mesh-repeats`, 100 by default in the script) to reach more than 100 samples.

### 1. Run on each machine

Close other heavy programs first, and plug in laptops. Pick a short `--label`
that identifies the machine (it becomes the folder name):

```bash
tools/run_profile.sh --label pc-<name> --scans data/ncd_sample/scans --icp-voxel 1.0
```

This writes `results/<label>/` with `timings.csv`, `mesh.csv`, `run.log` and
`system.txt` (CPU, memory, OS, compiler, commit, CPU governor, load average).
It takes about 3 minutes on a desktop CPU; on the Kria or the Jetson it can take
several times longer, mostly because of the 100 mesh repetitions. Use
`--mesh-repeats 101` as the minimum that still meets the requirement.

Commit the `results/<label>/` folder (the mesh `.ply` is ignored by git).

### 2. Analyze

```bash
python3 tools/analyze_profile.py results
```

It reads every `results/*/timings.csv` and writes to `results/analysis/`:

| File | Content |
|---|---|
| `summary.md` | Per machine: n, mean, std, 95 % CI, min, p50, p95, max, share of the scan time and CPU/wall ratio for each stage, plus memory and throughput. A cross-machine comparison table when there is more than one machine. |
| `summary.csv` | The same statistics in tabular form |
| `fig_breakdown.png` | Mean time per scan split by stage, one bar per machine |
| `fig_stages_box.png` | Distribution of every stage per machine (log scale) |
| `fig_timeline_<label>.png` | Time per scan and resident memory along the sequence |

`--skip-first N` drops the first `N` scans of every machine as warm-up
(the first registrations are cheaper because the local map is still empty).
If you use it, state it in the paper.


## Viewing the mesh

We use [MeshLab](https://www.meshlab.net/) to inspect the reconstruction.

Fedora:

```bash
sudo dnf install meshlab
```

Ubuntu:

```bash
sudo apt install meshlab
```

Open the mesh:

```bash
meshlab <out>.ply
```


## Expected results




## Usage of AI

Note on the use of AI tools

AI tools are used as support to understand concepts, generate ideas, and improve the writing of the documentation. Implementation, validation, and results are the student's responsibility, who assumes responsibility for any use beyond, or not described in, the above.

The shared conversation links are attached as evidence.

Gerson: https://claude.ai/share/be045057-e15d-462e-9e98-40855a3e21fa

## References

- I. Vizzo, T. Guadagnino, B. Mersch, L. Wiesmann, J. Behley, and C. Stachniss,
  "KISS-ICP: In Defense of Point-to-Point ICP – Simple, Accurate, and Robust
  Registration If Done the Right Way," *IEEE Robotics and Automation Letters*,
  vol. 8, no. 2, pp. 1029–1036, 2023.
- I. Vizzo, T. Guadagnino, J. Behley, and C. Stachniss, "VDBFusion: Flexible and
  Efficient TSDF Integration of Range Sensor Data," *Sensors*, vol. 22, no. 3,
  p. 1296, 2022.
- M. Ramezani, Y. Wang, M. Camurri, D. Wisth, M. Mattamala, and M. Fallon,
  "The Newer College Dataset: Handheld LiDAR, Inertial and Vision with Ground
  Truth," in *Proc. IEEE/RSJ IROS*, 2020. Licensed under CC BY-NC-SA 4.0.

## Authors

Gonzalo Alpízar Salas, Gerson Adrián Cordero Zúñiga, Nicole Irina Corrales
Rodríguez, Keilin Loáisiga Téllez — Escuela de Ingeniería Electrónica, ITCR.
