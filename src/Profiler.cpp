#include "recon/Profiler.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <fstream>

namespace recon {
namespace {

std::timespec ProcessCpuNow() {
    std::timespec ts{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return ts;
}

double DiffMs(const std::timespec& a, const std::timespec& b) {
    return static_cast<double>(b.tv_sec - a.tv_sec) * 1e3 +
           static_cast<double>(b.tv_nsec - a.tv_nsec) * 1e-6;
}

}  // namespace

void StageTimer::Start() {
    cpu0_ = ProcessCpuNow();
    wall0_ = std::chrono::steady_clock::now();
}

StageTime StageTimer::Stop() const {
    const auto wall1 = std::chrono::steady_clock::now();
    const std::timespec cpu1 = ProcessCpuNow();
    StageTime t;
    t.wall_ms = std::chrono::duration<double, std::milli>(wall1 - wall0_).count();
    t.cpu_ms = DiffMs(cpu0_, cpu1);
    return t;
}

std::size_t CurrentRssKb() {
    // statm: size resident shared text lib data dt  (en paginas)
    std::ifstream statm("/proc/self/statm");
    std::size_t size_pages = 0, resident_pages = 0;
    if (!(statm >> size_pages >> resident_pages)) return 0;
    const long page_kb = sysconf(_SC_PAGESIZE) / 1024;
    return resident_pages * static_cast<std::size_t>(page_kb);
}

std::size_t PeakRssKb() {
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) return 0;
    return static_cast<std::size_t>(usage.ru_maxrss);  // Linux: ya viene en KiB
}

}  // namespace recon
