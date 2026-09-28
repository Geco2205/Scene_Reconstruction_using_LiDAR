#pragma once

#include <chrono>
#include <cstddef>
#include <ctime>

namespace recon {

// Tiempo de una etapa: pared y CPU, en milisegundos.
struct StageTime {
    double wall_ms = 0.0;
    double cpu_ms = 0.0;
};

// Cronometro por etapa. El cociente CPU/pared da cuantos nucleos uso la
// etapa: ~1.0 serial, >1.0 paralela (KISS-ICP usa TBB internamente).
class StageTimer {
public:
    void Start();
    StageTime Stop() const;

private:
    std::chrono::steady_clock::time_point wall0_{};
    std::timespec cpu0_{};
};

// RSS actual del proceso en KiB, medido al cierre de cada scan.
std::size_t CurrentRssKb();

// RSS pico del proceso en KiB.
std::size_t PeakRssKb();

}  // namespace recon
