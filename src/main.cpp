// main.cpp -- pipeline de reconstruccion volumetrica en CPU.
//
//   PCD -> filtro de rango (NEON) -> KISS-ICP (pose) -> VDBFusion (TSDF) -> PLY
//
// KISS-ICP aporta el registro 3D en CPU y VDBFusion la integracion TSDF y la
// extraccion de malla. Este archivo los conecta y cronometra cada etapa
// (ver Profiler.hpp).

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <kiss_icp/pipeline/KissICP.hpp>
#include <vdbfusion/VDBVolume.h>

#include "recon/Io.hpp"
#include "recon/PcdReader.hpp"
#include "recon/Profiler.hpp"
#include "recon/RangeFilter.hpp"

namespace {

struct Options {
    std::string scan_dir;
    std::string output = "mesh.ply";
    std::size_t max_scans = 0;     // 0 = todos
    double tsdf_voxel = 0.10;      // m, resolucion de la rejilla TSDF
    double icp_voxel = 0.50;       // m, submuestreo interno de KISS-ICP
    double min_range = 1.0;        // m, descarta retornos del propio sensor
    double max_range = 60.0;       // m
    std::string timing_csv;        // vacio = no escribir
    std::string mesh_csv;          // vacio = no escribir
    std::size_t mesh_repeats = 1;  // veces que se repite la extraccion de malla
};

void PrintUsage(const char* argv0) {
    std::cout
        << "Uso: " << argv0 << " --scans <dir> [opciones]\n\n"
        << "  --scans <dir>        Directorio con los archivos .pcd  (obligatorio)\n"
        << "  --out <archivo>      Malla de salida en PLY            (mesh.ply)\n"
        << "  --max-scans <n>      Maximo de scans a procesar, 0=todos (0)\n"
        << "  --tsdf-voxel <m>     Tamano de voxel de la TSDF        (0.10)\n"
        << "  --icp-voxel <m>      Tamano de voxel de KISS-ICP       (0.50)\n"
        << "  --min-range <m>      Rango minimo del filtro           (1.0)\n"
        << "  --max-range <m>      Rango maximo del filtro           (60.0)\n"
        << "  --timing-csv <arch>  Guarda tiempos y memoria por scan en CSV\n"
        << "  --mesh-repeats <n>   Repite la extraccion de malla n veces (1)\n"
        << "  --mesh-csv <arch>    Guarda el tiempo de cada extraccion en CSV\n";
}

bool ParseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Falta el valor de " + arg);
            return argv[++i];
        };
        if (arg == "--scans")              opt.scan_dir     = next();
        else if (arg == "--out")           opt.output       = next();
        else if (arg == "--max-scans")     opt.max_scans    = std::stoull(next());
        else if (arg == "--tsdf-voxel")    opt.tsdf_voxel   = std::stod(next());
        else if (arg == "--icp-voxel")     opt.icp_voxel    = std::stod(next());
        else if (arg == "--min-range")     opt.min_range    = std::stod(next());
        else if (arg == "--max-range")     opt.max_range    = std::stod(next());
        else if (arg == "--timing-csv")    opt.timing_csv   = next();
        else if (arg == "--mesh-csv")      opt.mesh_csv     = next();
        else if (arg == "--mesh-repeats")  opt.mesh_repeats = std::max<std::size_t>(1, std::stoull(next()));
        else if (arg == "-h" || arg == "--help") { PrintUsage(argv[0]); return false; }
        else throw std::runtime_error("Argumento desconocido: " + arg);
    }
    if (opt.scan_dir.empty()) { PrintUsage(argv[0]); return false; }
    return true;
}

// Etapas por scan, en orden de ejecucion. El nombre es el prefijo de las
// columnas del CSV.
enum Stage { kRead, kFilter, kConvert, kRegister, kTransform, kIntegrate, kNumStages };
constexpr const char* kStageNames[kNumStages] = {
    "read", "filter", "convert", "register", "transform", "integrate"};

// Fila del CSV de tiempos: una por scan procesado.
struct ScanSample {
    std::size_t scan_index = 0;   // posicion del archivo en la secuencia ordenada
    std::size_t points_in = 0;
    std::size_t points_kept = 0;
    recon::StageTime stage[kNumStages];
    std::size_t rss_kb = 0;       // memoria residente al terminar el scan
};

struct Summary {
    double mean = 0, stddev = 0, p50 = 0, p95 = 0, min = 0, max = 0;
};

// Estadisticos del resumen de consola. El resto sale de analyze_profile.py.
Summary Summarize(std::vector<double> v) {
    Summary s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    const double n = static_cast<double>(v.size());
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / n;
    double sq = 0;
    for (double x : v) sq += (x - s.mean) * (x - s.mean);
    s.stddev = v.size() > 1 ? std::sqrt(sq / (n - 1)) : 0.0;
    auto pct = [&](double p) {  // percentil por el metodo del rango mas cercano
        const auto idx = static_cast<std::size_t>(std::ceil(p * n));
        return v[std::min(idx == 0 ? 0 : idx - 1, v.size() - 1)];
    };
    s.p50 = pct(0.50);
    s.p95 = pct(0.95);
    s.min = v.front();
    s.max = v.back();
    return s;
}

void PrintSummaryRow(const std::string& name, const Summary& s) {
    std::cout << "  " << std::left << std::setw(11) << name << std::right
              << std::setw(9) << s.mean << std::setw(9) << s.stddev
              << std::setw(9) << s.p50 << std::setw(9) << s.p95
              << std::setw(9) << s.max << "\n";
}

}  // namespace

int main(int argc, char** argv) try {
    Options opt;
    if (!ParseArgs(argc, argv, opt)) return EXIT_FAILURE;

    // --- Configuracion de KISS-ICP -------------------------------------
    // Solo se tocan los campos estables del API. Si el header de la version
    // que fijaron en CMake expone otros, revisen KissICP.hpp antes de agregar.
    kiss_icp::pipeline::KISSConfig config;
    config.voxel_size = opt.icp_voxel;
    config.max_range = opt.max_range;
    config.min_range = opt.min_range;
    config.deskew = false;  // no extraemos el timestamp por punto del PCD

    kiss_icp::pipeline::KissICP odometry(config);

    // --- Configuracion de VDBFusion ------------------------------------
    // sdf_trunc = 4 * voxel es la regla habitual: la banda de truncamiento
    // debe cubrir varios voxeles para que el promedio ponderado tenga efecto.
    const double sdf_trunc = 4.0 * opt.tsdf_voxel;
    const bool space_carving = false;
    vdbfusion::VDBVolume tsdf_volume(opt.tsdf_voxel, sdf_trunc, space_carving);

    const auto scans = recon::ListScansSorted(opt.scan_dir);
    if (scans.empty()) {
        std::cerr << "No se encontraron archivos .pcd en " << opt.scan_dir << "\n";
        return EXIT_FAILURE;
    }
    const std::size_t n_scans =
        (opt.max_scans == 0) ? scans.size() : std::min(opt.max_scans, scans.size());

    std::cout << "Scans encontrados : " << scans.size() << "  (se procesan " << n_scans << ")\n"
              << "Voxel TSDF / ICP  : " << opt.tsdf_voxel << " m / " << opt.icp_voxel << " m\n"
              << "Rango             : [" << opt.min_range << ", " << opt.max_range << "] m\n"
              << "Filtro            : " << (recon::NeonEnabled() ? "NEON" : "escalar") << "\n"
              << "Compilador        : " << __VERSION__ << "\n\n";

    std::vector<ScanSample> samples;
    samples.reserve(n_scans);

    recon::StageTimer timer;
    recon::PointCloudSoA filtered;

    for (std::size_t i = 0; i < n_scans; ++i) {
        ScanSample s;
        s.scan_index = i;

        timer.Start();
        const recon::PointCloudSoA raw = recon::ReadPcd(scans[i]);
        s.stage[kRead] = timer.Stop();
        s.points_in = raw.size();

        timer.Start();
        recon::FilterByRange(raw, static_cast<float>(opt.min_range),
                             static_cast<float>(opt.max_range), filtered);
        s.stage[kFilter] = timer.Stop();
        s.points_kept = filtered.size();

        if (filtered.empty()) {
            std::cerr << "  [" << i << "] scan vacio tras el filtro, se omite\n";
            continue;
        }

        timer.Start();
        const std::vector<Eigen::Vector3d> points = recon::ToEigen(filtered);
        s.stage[kConvert] = timer.Stop();

        // --- Registro ---------------------------------------------------
        // RegisterFrame actualiza el mapa interno y la pose. Se ignora el par
        // que devuelve (source, frame_downsample) porque para la integracion
        // usamos la nube filtrada completa, no la submuestreada por el ICP.
        timer.Start();
        odometry.RegisterFrame(points);
        const Sophus::SE3d pose = odometry.pose();
        s.stage[kRegister] = timer.Stop();

        // --- Transformacion al marco global ----------------------------
        // Antes caia dentro del timer de la integracion.
        timer.Start();
        std::vector<Eigen::Vector3d> global_points;
        global_points.reserve(points.size());
        for (const auto& p : points) global_points.emplace_back(pose * p);
        const Eigen::Vector3d origin = pose.translation();
        s.stage[kTransform] = timer.Stop();

        // --- Integracion TSDF -------------------------------------------
        // VDBFusion espera los puntos ya en el marco global y el origen del
        // sensor, que usa para trazar los rayos y marcar el espacio libre.
        timer.Start();
        tsdf_volume.Integrate(global_points, origin, [](float /*sdf*/) { return 1.0f; });
        s.stage[kIntegrate] = timer.Stop();

        s.rss_kb = recon::CurrentRssKb();
        samples.push_back(s);

        if (i % 10 == 0 || i + 1 == n_scans) {
            double total = 0;
            for (const auto& st : s.stage) total += st.wall_ms;
            std::cout << "  [" << std::setw(4) << i << "] "
                      << std::setw(7) << s.points_in << " -> "
                      << std::setw(7) << s.points_kept << " pts   "
                      << std::fixed << std::setprecision(1) << std::setw(7) << total << " ms   "
                      << std::setw(6) << s.rss_kb / 1024 << " MiB\n";
        }
    }

    // --- Extraccion de malla -------------------------------------------
    // Ocurre una vez por corrida; se repite sobre el mismo volumen para
    // tener muestras.
    std::cout << "\nExtrayendo malla (" << opt.mesh_repeats << " repeticion(es))...\n";
    std::vector<recon::StageTime> mesh_times;
    mesh_times.reserve(opt.mesh_repeats);
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    for (std::size_t r = 0; r < opt.mesh_repeats; ++r) {
        timer.Start();
        std::tie(vertices, triangles) = tsdf_volume.ExtractTriangleMesh();
        mesh_times.push_back(timer.Stop());
    }

    timer.Start();
    recon::WritePly(opt.output, vertices, triangles);
    const recon::StageTime write_time = timer.Stop();

    std::cout << "Malla: " << vertices.size() << " vertices, " << triangles.size()
              << " triangulos\n"
              << "Guardada en: " << opt.output << "  (escritura "
              << std::fixed << std::setprecision(1) << write_time.wall_ms << " ms)\n";

    // --- Resumen -------------------------------------------------------
    if (!samples.empty()) {
        std::cout << "\nTiempo de pared por etapa (" << samples.size() << " muestras, ms):\n"
                  << "  etapa          media      std      p50      p95      max\n"
                  << std::fixed << std::setprecision(2);
        std::vector<double> total(samples.size(), 0.0);
        for (int st = 0; st < kNumStages; ++st) {
            std::vector<double> v;
            v.reserve(samples.size());
            for (std::size_t k = 0; k < samples.size(); ++k) {
                v.push_back(samples[k].stage[st].wall_ms);
                total[k] += samples[k].stage[st].wall_ms;
            }
            PrintSummaryRow(kStageNames[st], Summarize(v));
        }
        PrintSummaryRow("TOTAL/scan", Summarize(total));

        std::vector<double> mesh_wall;
        for (const auto& m : mesh_times) mesh_wall.push_back(m.wall_ms);
        std::cout << "\nExtraccion de malla (" << mesh_times.size() << " muestras, ms):\n";
        PrintSummaryRow("mesh", Summarize(mesh_wall));

        std::cout << "\nMemoria: RSS final " << recon::CurrentRssKb() / 1024
                  << " MiB, pico " << recon::PeakRssKb() / 1024 << " MiB\n";
    }

    if (!opt.timing_csv.empty()) {
        std::ofstream csv(opt.timing_csv);
        csv << "scan,points_in,points_kept";
        for (const char* name : kStageNames) csv << "," << name << "_ms";
        for (const char* name : kStageNames) csv << "," << name << "_cpu_ms";
        csv << ",rss_kb\n";
        csv << std::defaultfloat << std::setprecision(6);
        for (const auto& s : samples) {
            csv << s.scan_index << "," << s.points_in << "," << s.points_kept;
            for (const auto& st : s.stage) csv << "," << st.wall_ms;
            for (const auto& st : s.stage) csv << "," << st.cpu_ms;
            csv << "," << s.rss_kb << "\n";
        }
        std::cout << "Tiempos por scan en: " << opt.timing_csv << "\n";
    }

    if (!opt.mesh_csv.empty()) {
        std::ofstream csv(opt.mesh_csv);
        csv << "repeat,mesh_ms,mesh_cpu_ms,vertices,triangles\n";
        csv << std::defaultfloat << std::setprecision(6);
        for (std::size_t r = 0; r < mesh_times.size(); ++r) {
            csv << r << "," << mesh_times[r].wall_ms << "," << mesh_times[r].cpu_ms << ","
                << vertices.size() << "," << triangles.size() << "\n";
        }
        std::cout << "Tiempos de malla en: " << opt.mesh_csv << "\n";
    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
}
