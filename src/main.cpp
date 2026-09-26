// main.cpp -- pipeline de reconstruccion volumetrica en CPU.
//
//   PCD -> filtro de rango (NEON) -> KISS-ICP (pose) -> VDBFusion (TSDF) -> PLY
//
// KISS-ICP aporta el registro 3D en CPU y VDBFusion la integracion TSDF y la
// extraccion de malla. Este archivo solo los conecta y cronometra cada etapa.

#include <Eigen/Core>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <kiss_icp/pipeline/KissICP.hpp>
#include <vdbfusion/VDBVolume.h>

#include "recon/Io.hpp"
#include "recon/PcdReader.hpp"
#include "recon/RangeFilter.hpp"

namespace {

struct Options {
    std::string scan_dir;
    std::string output = "mesh.ply";
    std::size_t max_scans = 0;   // 0 = todos
    double tsdf_voxel = 0.10;    // m, resolucion de la rejilla TSDF
    double icp_voxel = 0.50;     // m, submuestreo interno de KISS-ICP
    double min_range = 1.0;      // m, descarta retornos del propio sensor
    double max_range = 60.0;     // m
    std::string timing_csv;      // vacio = no escribir
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
        << "  --timing-csv <arch>  Guarda los tiempos por etapa en CSV\n";
}

bool ParseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Falta el valor de " + arg);
            return argv[++i];
        };
        if (arg == "--scans")            opt.scan_dir   = next();
        else if (arg == "--out")         opt.output     = next();
        else if (arg == "--max-scans")   opt.max_scans  = std::stoull(next());
        else if (arg == "--tsdf-voxel")  opt.tsdf_voxel = std::stod(next());
        else if (arg == "--icp-voxel")   opt.icp_voxel  = std::stod(next());
        else if (arg == "--min-range")   opt.min_range  = std::stod(next());
        else if (arg == "--max-range")   opt.max_range  = std::stod(next());
        else if (arg == "--timing-csv")  opt.timing_csv = next();
        else if (arg == "-h" || arg == "--help") { PrintUsage(argv[0]); return false; }
        else throw std::runtime_error("Argumento desconocido: " + arg);
    }
    if (opt.scan_dir.empty()) { PrintUsage(argv[0]); return false; }
    return true;
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
              << "Filtro            : " << (recon::NeonEnabled() ? "NEON" : "escalar") << "\n\n";

    // Una fila por scan; alimenta el requisito de >100 muestras por etapa.
    struct Timing {
        double read_ms, filter_ms, convert_ms, register_ms, integrate_ms;
        std::size_t points_in, points_kept;
    };
    std::vector<Timing> timings;
    timings.reserve(n_scans);

    recon::Stopwatch sw;
    recon::PointCloudSoA filtered;

    for (std::size_t i = 0; i < n_scans; ++i) {
        Timing t{};

        sw.Start();
        const recon::PointCloudSoA raw = recon::ReadPcd(scans[i]);
        t.read_ms = sw.ElapsedMs();
        t.points_in = raw.size();

        sw.Start();
        recon::FilterByRange(raw, static_cast<float>(opt.min_range),
                             static_cast<float>(opt.max_range), filtered);
        t.filter_ms = sw.ElapsedMs();
        t.points_kept = filtered.size();

        if (filtered.empty()) {
            std::cerr << "  [" << i << "] scan vacio tras el filtro, se omite\n";
            continue;
        }

        sw.Start();
        const std::vector<Eigen::Vector3d> points = recon::ToEigen(filtered);
        t.convert_ms = sw.ElapsedMs();

        // --- Registro ---------------------------------------------------
        // RegisterFrame actualiza el mapa interno y la pose. Se ignora el par
        // que devuelve (source, frame_downsample) porque para la integracion
        // usamos la nube filtrada completa, no la submuestreada por el ICP.
        sw.Start();
        odometry.RegisterFrame(points);
        const Sophus::SE3d pose = odometry.pose();
        t.register_ms = sw.ElapsedMs();

        // --- Integracion TSDF -------------------------------------------
        // VDBFusion espera los puntos ya en el marco global y el origen del
        // sensor, que usa para trazar los rayos y marcar el espacio libre.
        sw.Start();
        std::vector<Eigen::Vector3d> global_points;
        global_points.reserve(points.size());
        for (const auto& p : points) global_points.emplace_back(pose * p);
        const Eigen::Vector3d origin = pose.translation();

        tsdf_volume.Integrate(global_points, origin, [](float /*sdf*/) { return 1.0f; });
        t.integrate_ms = sw.ElapsedMs();

        timings.push_back(t);

        if (i % 10 == 0 || i + 1 == n_scans) {
            const double total = t.read_ms + t.filter_ms + t.convert_ms +
                                 t.register_ms + t.integrate_ms;
            std::cout << "  [" << std::setw(4) << i << "] "
                      << std::setw(7) << t.points_in << " -> "
                      << std::setw(7) << t.points_kept << " pts   "
                      << std::fixed << std::setprecision(1) << std::setw(7) << total << " ms\n";
        }
    }

    // --- Extraccion de malla -------------------------------------------
    std::cout << "\nExtrayendo malla...\n";
    sw.Start();
    auto [vertices, triangles] = tsdf_volume.ExtractTriangleMesh();
    const double mesh_ms = sw.ElapsedMs();

    recon::WritePly(opt.output, vertices, triangles);
    std::cout << "Malla: " << vertices.size() << " vertices, " << triangles.size()
              << " triangulos  (" << std::fixed << std::setprecision(1) << mesh_ms << " ms)\n"
              << "Guardada en: " << opt.output << "\n";

    // --- Resumen de tiempos ---------------------------------------------
    if (!timings.empty()) {
        Timing sum{};
        for (const auto& t : timings) {
            sum.read_ms += t.read_ms;
            sum.filter_ms += t.filter_ms;
            sum.convert_ms += t.convert_ms;
            sum.register_ms += t.register_ms;
            sum.integrate_ms += t.integrate_ms;
        }
        const double n = static_cast<double>(timings.size());
        std::cout << "\nPromedio por scan (" << timings.size() << " muestras):\n"
                  << std::fixed << std::setprecision(2)
                  << "  lectura     " << sum.read_ms / n << " ms\n"
                  << "  filtro      " << sum.filter_ms / n << " ms\n"
                  << "  conversion  " << sum.convert_ms / n << " ms\n"
                  << "  registro    " << sum.register_ms / n << " ms\n"
                  << "  integracion " << sum.integrate_ms / n << " ms\n"
                  << "  TOTAL       "
                  << (sum.read_ms + sum.filter_ms + sum.convert_ms +
                      sum.register_ms + sum.integrate_ms) / n << " ms\n";
    }

    if (!opt.timing_csv.empty()) {
        std::ofstream csv(opt.timing_csv);
        csv << "scan,points_in,points_kept,read_ms,filter_ms,convert_ms,register_ms,integrate_ms\n";
        for (std::size_t i = 0; i < timings.size(); ++i) {
            const auto& t = timings[i];
            csv << i << "," << t.points_in << "," << t.points_kept << ","
                << t.read_ms << "," << t.filter_ms << "," << t.convert_ms << ","
                << t.register_ms << "," << t.integrate_ms << "\n";
        }
        std::cout << "Tiempos por scan en: " << opt.timing_csv << "\n";
    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
}
