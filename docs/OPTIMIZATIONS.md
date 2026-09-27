# Optimizations of the CPU prototype

This document records which optimizations were applied to the CPU prototype,
which were not, and why. It follows the methodology of Chapter 2 of the course
notes (*Sistemas multiprocesador y su programación*): measure the baseline,
profile, find where the time goes, optimize, and measure again. Every
optimization can be switched on or off on its own, so each one is compared
against the same baseline.

## Summary

| Optimization | Status | Switch | Effect on throughput (2-core VM, preliminary) |
|---|---|---|---|
| Task parallelism, `prefetch` (read and filter scan *k+1* while scan *k* is processed) | Applied | `--pipeline prefetch` | +1 % (bounded by Amdahl: the stage it hides is < 2 % of the time) |
| Task parallelism, `full` (also integrate scan *k* while scan *k+1* is registered) | Applied | `--pipeline full` | **+27 %** (10.55 → 13.44 scans/s) |
| `-march=native` / `-mcpu=native` | Applied | `-DRECON_OPT_NATIVE=ON` | +2 % |
| Link-Time Optimization (`-flto`) | Applied | `-DRECON_OPT_LTO=ON` | +2 % |
| 64-byte aligned SoA + branch-free compaction | Applied | `-DRECON_OPT_SOA_ALIGNED=ON` | +1 % overall; `read` −44 %, `filter` −48 % |
| NEON intrinsics in the range filter | Already in the prototype | `-DRECON_OPT_NEON=ON` (default) | To be measured on the Kria/Jetson (`noneon` variant) |
| Thread affinity (pinning) | Not applied | — | See [Not applied](#not-applied) |
| `-ffast-math` / `-Ofast` | Not applied | — | See [Not applied](#not-applied) |
| Profile-Guided Optimization | Not applied | — | See [Not applied](#not-applied) |
| Manual SIMD in KISS-ICP / VDBFusion | Not applied | — | Left for the GPU/FPGA stages |

The numbers in this table come from a validation run on a 2-core cloud VM
(Intel Xeon @ 2.10 GHz), used to check that every variant builds, runs and
produces the same mesh. They are **not** the project results: the tables in
[Results per machine](#results-per-machine) are filled with the runs on the
team's PCs and the embedded board.

## Methodology

- **Baseline**: default build (`RECON_OPT_*` switches `OFF`, NEON `ON` as in the
  original prototype) and `--pipeline off`. It runs exactly the code of the
  profiling branch, split into three functions (load, register, integrate) that
  all variants share.
- **One change at a time**: each variant differs from the baseline in a single
  switch. The `all` variant combines every switch.
- **Data**: the 300-scan Newer College sample with `--icp-voxel 1.0`, which gives
  300 samples per per-scan stage and 101 samples of mesh extraction
  (more than 100 per stage, as the project requires).
- **Metric**: with `--pipeline` the stages overlap, so the sum of stage times is
  no longer the time per scan. The program writes `done_ms` (the moment each
  scan finished) and `analyze_profile.py` reports the **period** between
  consecutive scans; throughput is `1000 / period`. In sequential mode both
  coincide.
- **Correctness**: the output mesh of every variant is compared against the
  original code (see [Correctness](#correctness)).
- **Tool**: `tools/run_optimizations.sh` builds each variant, runs it through
  `tools/run_profile.sh` and writes the comparison table.

## Where the time goes

Baseline on PC1 (Intel i3-1115G4, 4 threads), from `results/pc1-nicole/`:

| Stage | Mean (ms) | Share |
|---|---:|---:|
| read | 0.54 | 0.9 % |
| filter | 0.17 | 0.3 % |
| convert | 0.12 | 0.2 % |
| register (KISS-ICP) | 31.15 | 51.3 % |
| transform | 0.96 | 1.6 % |
| integrate (VDBFusion) | 27.73 | 45.7 % |
| **total per scan** | **60.68** | |
| mesh extraction (once per sequence) | 686.48 | — |

Two stages, registration and TSDF integration, take 97 % of the time, and both
run inside third-party libraries. That shapes everything below:

- An optimization of our own code (reader, filter, conversion) can save at most
  the ~1.4 % those stages take. By Amdahl's law the best possible speedup from
  them is `1 / (1 − 0.014) ≈ 1.014`.
- The optimizations that can matter are the ones that reach the libraries
  (compiler flags applied to them) or that run the two heavy stages at the same
  time (task parallelism).

## Applied

### 1. Task parallelism (`--pipeline`)

*Chapter 2: heterogeneous tasks, producer-consumer queues, ping-pong buffers.*

The pipeline is a chain of heterogeneous tasks, like the image-processing
example of Fig. 2.17: each scan goes through load → register → integrate. The
stages of different scans are independent except for one ordering constraint:
KISS-ICP must receive the scans in order, because each registration uses the
local map built from the previous ones. The TSDF volume only needs each scan's
pose, which is known as soon as registration ends.

Each stage runs in its own thread and consecutive stages are connected by a
bounded queue of capacity 2 (`include/recon/BoundedQueue.hpp`: a mutex and two
condition variables). Capacity 2 makes it a ping-pong buffer: while one scan is
being processed the next one is filled, and a stage that gets ahead blocks
instead of piling scans in memory. Each queue is FIFO and each stage has a
single thread, so the scan order is preserved and the result is identical to
the sequential run.

Two modes are provided:

| Mode | Threads | What overlaps |
|---|---|---|
| `prefetch` | 2 | Reading, filtering and converting scan *k+1* overlap with registering and integrating scan *k*. This is what the task plan asked for. |
| `full` | 3 | Additionally, integrating scan *k* overlaps with registering scan *k+1*. |

**Why two modes.** `prefetch` hides a stage that takes ~1.4 % of the time, so
its ceiling is about 1.4 %; it was kept because it isolates that effect.
`full` overlaps the two heavy stages. Its ideal period is the slowest stage
instead of the sum: `max(31.15 + 0.96, 27.73) = 32.1 ms` instead of 60.7 ms on
PC1, a theoretical ceiling of **1.89×**.

**Measured (2-core VM).** `prefetch`: 1.01×. `full`: 1.27× (94.8 → 74.4 ms
period). The gap to the ceiling comes from contention: KISS-ICP already uses
TBB to spread registration over the cores (CPU/wall of `register` is 1.9 on the
2-core VM and 3.7 on the 4-thread PC1, while `integrate` is serial at 1.0), so
when integration runs at the same time both stages compete for the same cores. With `full`, `register` goes from 55.7 to 71.9 ms and `integrate` from
36.0 to 44.4 ms, yet the period still drops because they now overlap. On
machines with more cores the gain should be larger; that is what the runs on
the PCs will show.

**Cost.** Latency per scan grows (a scan spends more time in the pipeline, and
each stage is slower under contention) while throughput improves. For offline
reconstruction throughput is what matters. Peak memory stays in the same range
(two extra scans in flight are ~3 MB each).

**Measurement caveat.** In pipeline mode each stage measures CPU time with
`CLOCK_THREAD_CPUTIME_ID`, because the process clock would mix the concurrent
stages. That clock does not see TBB's worker threads, so `register_cpu_ms`
underestimates the real CPU of registration in pipeline mode. Wall times and the
period are exact in every mode.

### 2. `-march=native` / `-mcpu=native` (`RECON_OPT_NATIVE`)

*Chapter 2: optimization for the microarchitecture.*

Lets the compiler use every instruction-set extension of the machine it builds
on (AVX2 and FMA on the PCs, the exact Cortex-A53/A57 tuning on ARM) and tunes
instruction scheduling for it. The flag is applied **before** `FetchContent`,
so it also compiles KISS-ICP and VDBFusion, where the time is spent. On ARM,
GCC uses `-mcpu=native` (sets both architecture and tuning). Previously
`-mcpu=native` was always on for ARM; it now belongs to this switch so the ARM
baseline is comparable.

**Measured (2-core VM):** 1.02×. `transform` improves 24 % (Eigen uses AVX for
the 3D transform), but `register` and `integrate` barely change. The heavy
inner loops live in libraries that are **not** recompiled: TBB and OpenVDB come
precompiled from the system packages, and VDBFusion's integration is dominated
by OpenVDB's tree accesses. KISS-ICP's registration is dominated by hash-map
lookups for nearest neighbors, which are limited by memory latency, not by
arithmetic.

**Caveats:** the binary only runs on CPUs with the same extensions (not
portable, as the chapter warns), and FMA contraction changes rounding: the mesh
differs by 1 triangle out of ~930 000 on 60 scans. The difference is numeric,
not a bug, and it is why this switch is off in the baseline.

### 3. Link-Time Optimization (`RECON_OPT_LTO`)

*Chapter 2: optimization across compilation units.*

Enables `-flto` through CMake's `CMAKE_INTERPROCEDURAL_OPTIMIZATION`, applied to
our code and to the KISS-ICP and VDBFusion libraries, so the compiler can inline
across files (for example, Sophus and Eigen calls between our pipeline and
KISS-ICP). **Measured (2-core VM):** 1.02×. As with `native`, it cannot reach
into TBB and OpenVDB, which are shared libraries from the system. GCC emits a
`-Walloc-size-larger-than` warning from an OpenVDB header during the LTO link;
it is a known false positive of that header and does not affect the result.

### 4. Aligned SoA and branch-free compaction (`RECON_OPT_SOA_ALIGNED`)

*Chapter 2: alignment and cache-oriented programming, SoA vs AoS, transforming
control flow.*

The point cloud was already stored as a structure of arrays (`PointCloudSoA`,
one array per axis), which is what makes the NEON filter possible. This switch
adds:

1. **64-byte alignment** of the three arrays through an aligned allocator
   (`include/recon/PcdReader.hpp`). 64 B is one cache line on both the PCs and
   the Cortex-A cores, so vector loads never straddle two lines. The filter
   tells the compiler about it with `__builtin_assume_aligned`.
2. **Indexed, branch-free writes** in the PCD reader and the range filter. The
   output is sized once and every point is written at index `k`, which advances
   by 1 only if the point passes (`k += keep`). This removes the capacity check
   of `push_back` on every point and the branch whose outcome is data dependent
   (hard to predict). The allocator skips zero-initialization on `resize`, so
   sizing the output does not cost an extra pass.

**Measured (2-core VM):** `read` 1.20 → 0.67 ms (−44 %) and `filter`
0.29 → 0.15 ms (−48 %), but the whole scan only improves by 1 %, exactly what
the Amdahl bound predicted.

**Autovectorization.** GCC's report (`-fopt-info-vec-missed`) shows that the
filter loop is not autovectorized in either version on x86: in the original
because of the branch (*control flow in loop*), and in the new one because the
output index depends on the data. A compaction with variable-size output
cannot be expressed as a plain vector loop without a compress-store
instruction (AVX-512 has it, AVX2 and ARMv8 NEON do not). That is why the
arithmetic is vectorized by hand with NEON on ARM and the write stays scalar.

### 5. NEON in the range filter (`RECON_OPT_NEON`, already in the prototype)

The range filter computes `x² + y² + z²` and both range comparisons four points
at a time with NEON intrinsics (`src/RangeFilter.cpp`). This was part of the
prototype before the optimization work; the new switch only allows turning it
off (`-DRECON_OPT_NEON=OFF`) to measure its effect on ARM with the same code.
`tools/run_optimizations.sh` runs that `noneon` variant automatically on ARM.
On x86 the switch has no effect.

## Not applied

**Thread affinity (`pthread_setaffinity_np`).** The chapter recommends pinning
threads to avoid migrations and cold caches. It was not applied because most of
the work runs in TBB's thread pool inside KISS-ICP, which our code does not
control. Pinning our three pipeline threads to fixed cores would take those
cores away from TBB's scheduler and increase the contention measured in the
`full` mode. On the target machines (single socket, shared L3), migration cost
is also much smaller than in the multi-socket / NUMA systems the chapter
describes.

**NUMA-aware allocation.** All target machines (PCs, Kria, Jetson) have a single
socket and one memory controller, so there is no remote memory to avoid.

**`-ffast-math` / `-Ofast`.** They relax IEEE semantics (NaN/inf handling and
associativity). The reader drops non-finite points with `std::isfinite`, which
`-ffast-math` is allowed to optimize away, and ICP accumulates many sums whose
result would change with reassociation. The expected gain does not justify
losing reproducibility of the trajectory and the mesh.

**Profile-Guided Optimization.** PGO needs a two-step build with a
representative run, and it only reshapes code the compiler builds. As shown for
`native` and `LTO`, the code we can rebuild is not where the time is spent, so
the expected gain is of the same small order, at the cost of a more complex
build for every teammate and machine. It is left as optional.

**Manual SIMD (intrinsics) inside KISS-ICP or VDBFusion.** The two heavy stages
are the natural candidates, but their cost is dominated by irregular memory
access (a voxel hash map for nearest neighbors, and OpenVDB's sparse tree for
the TSDF), not by arithmetic that SIMD accelerates well. Modifying the
libraries would also break the "used as a library, pinned version" choice that
keeps the build reproducible. These stages are the targets of the GPU and FPGA
prototypes instead (see below).

**Structure of arrays in the conversion and transform.** KISS-ICP and VDBFusion
take `std::vector<Eigen::Vector3d>` (array of structures, double precision) in
their public API. Keeping SoA past the filter would require a conversion anyway,
so `convert` and `transform` stay AoS. Together they take ~1.8 % of the time.

**Parallelizing mesh extraction.** It runs once per sequence (~0.7 s on PC1)
inside VDBFusion and is serial. It does not affect per-scan throughput and it is
left for the accelerator stages.

## Correctness

On the first 60 scans of the sample, the mesh written by each variant was
compared against the original code of the profiling branch (`d5531cd`):

| Variant | Vertices | Triangles | Identical to original |
|---|---:|---:|---|
| original (`d5531cd`) | 550 186 | 930 148 | — |
| base, `--pipeline off` | 550 186 | 930 148 | yes (same MD5) |
| base, `--pipeline prefetch` | 550 186 | 930 148 | yes (same MD5) |
| base, `--pipeline full` | 550 186 | 930 148 | yes (same MD5) |
| `soa` | 550 186 | 930 148 | yes (same MD5) |
| `native` | 550 186 | 930 149 | no: 1 triangle, from FMA rounding |
| `all` + `--pipeline full` | 550 186 | 930 149 | no: same difference as `native` |

The pipeline produces bit-identical output, which confirms that the scan order
is preserved and that there are no data races between stages.

## Results per machine

Fill these tables from `results/opt-<label>/analysis/summary.md` after running
`tools/run_optimizations.sh` on each machine. Throughput in scans/s, speedup
against `base` on the same machine.

| Variant | PC1 (i3-1115G4) | PC2 | Kria KV260 / Jetson Nano |
|---|---:|---:|---:|
| base | | | |
| prefetch | | | |
| full | | | |
| native | | | |
| lto | | | |
| soa | | | |
| all | | | |
| noneon (ARM only) | — | — | |

Validation run on a 2-core cloud VM (Intel Xeon @ 2.10 GHz, 7.8 GiB, GCC 13.3),
for reference only:

| Variant | Period (ms) | Throughput (scans/s) | Speedup |
|---|---:|---:|---:|
| base | 94.79 | 10.55 | 1.00 |
| prefetch | 93.51 | 10.69 | 1.01 |
| full | 74.43 | 13.44 | 1.27 |
| native | 92.49 | 10.81 | 1.02 |
| lto | 92.83 | 10.77 | 1.02 |
| soa | 94.11 | 10.63 | 1.01 |
| all | 75.14 | 13.31 | 1.26 |

## Next steps

The profiling and these results point to the same conclusion: on the CPU, the
remaining time is in registration and TSDF integration, and compiler-level
optimizations barely reach them. The next prototypes should move those stages
to accelerators:

- **TSDF integration → GPU (CUDA, Jetson Nano).** Each point traces a ray
  through the voxels near the surface and updates them independently, which
  maps well onto thousands of GPU threads; the sparse storage has to be
  replaced by a GPU-friendly hash or dense block structure.
- **Registration → GPU or FPGA.** The nearest-neighbor search over the voxel
  hash map is the core of each ICP iteration and is repeated for every point;
  it is a candidate for a GPU kernel or for an FPGA pipeline with on-chip
  memory for the local map (HLS, Kria KV260).
- **Task pipeline across devices.** The `full` pipeline already separates
  registration and integration into independent stages connected by queues.
  In the heterogeneous prototype each stage can run on a different device
  (for example, registration on the CPU or FPGA and integration on the GPU)
  with the same structure.
