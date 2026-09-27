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
//
// Reloj de CPU:
//   kProcess (por defecto): CLOCK_PROCESS_CPUTIME_ID, suma todos los hilos. Es
//     el correcto en modo secuencial, porque incluye los hilos de TBB que usa
//     KISS-ICP.
//   kThread: CLOCK_THREAD_CPUTIME_ID, solo el hilo que llama. Se usa con
//     --pipeline, donde varias etapas corren a la vez y el reloj del proceso
//     mezclaria el CPU de todas. Limitacion: no ve los hilos de TBB, asi que en
//     modo pipeline register_cpu_ms subestima el CPU real del registro.
class StageTimer {
public:
    enum class CpuClock { kProcess, kThread };

    explicit StageTimer(CpuClock clock = CpuClock::kProcess) : clock_(clock) {}

    void Start();
    StageTime Stop() const;

private:
    std::chrono::steady_clock::time_point wall0_{};
    std::timespec cpu0_{};
    CpuClock clock_ = CpuClock::kProcess;
};

// RSS actual del proceso en KiB, medido al cierre de cada scan.
std::size_t CurrentRssKb();

// RSS pico del proceso en KiB.
std::size_t PeakRssKb();

}  // namespace recon
