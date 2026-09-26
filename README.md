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
| `--timing-csv <file>` | — | Write per-stage timings to a CSV file |

Scans are processed in timestamp order. File names follow
`cloud_<sec>_<nsec>.pcd` and are sorted numerically, because the nanosecond
field does not always have 9 digits.

## Outputs

- **`<out>.ply`**: triangle mesh (ASCII PLY). Open it with MeshLab or CloudCompare.
- **`<timing-csv>`**: one row per scan with columns
  `scan, points_in, points_kept, read_ms, filter_ms, convert_ms, register_ms, integrate_ms`.


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
