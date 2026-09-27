// Profiler.hpp -- medicion de tiempo y consumo de recursos por etapa.
//
// Cada etapa del pipeline se mide con dos relojes:
//   - pared (steady_clock): lo que tarda la etapa vista desde afuera.
//   - CPU del proceso (CLOCK_PROCESS_CPUTIME_ID): tiempo de CPU sumado de
//     TODOS los hilos del proceso durante la etapa.
//
// La razon cpu/pared indica cuantos nucleos estuvo usando la etapa en promedio.
// Una etapa serial da ~1.0; KISS-ICP usa TBB internamente, asi que el registro
// puede dar >1.0. Esto es lo que despues justifica que etapas se pueden (o ya
// estan) paralelizando.
//
// La memoria se lee de /proc/self/statm (RSS actual) y de getrusage (RSS pico).
// Ambas son especificas de Linux, que es el unico sistema que usa el proyecto
// (Fedora, Ubuntu, Kria, Jetson).

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

// Cronometro que mide pared y CPU del proceso a la vez.
class StageTimer {
public:
    void Start();
    StageTime Stop() const;

private:
    std::chrono::steady_clock::time_point wall0_{};
    std::timespec cpu0_{};
};

// RSS actual del proceso en KiB (memoria fisica en uso ahora mismo).
std::size_t CurrentRssKb();

// RSS pico del proceso en KiB desde que arranco.
std::size_t PeakRssKb();

}  // namespace recon
