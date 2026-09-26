// Io.hpp -- escritura de malla en PLY, listado ordenado de scans y cronometro.

#pragma once

#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

namespace recon {

// Escribe una malla triangular en PLY ascii.
// Se implementa a mano para no arrastrar Open3D como dependencia solo por esto.
void WritePly(const std::string& path,
              const std::vector<Eigen::Vector3d>& vertices,
              const std::vector<Eigen::Vector3i>& triangles);

// Lista los .pcd de un directorio ordenados por el timestamp del nombre.
//
// Los nombres del Newer College son del tipo cloud_<seg>_<nanoseg>.pcd. Ordenar
// como texto falla porque el campo de nanosegundos no siempre trae 9 digitos:
// "..._1820.pcd" quedaria antes que "..._999.pcd". Por eso se extraen todos los
// grupos de digitos del nombre y se comparan como numeros.
//
// La comparacion numerica resuelve bien el caso de ceros recortados a la
// izquierda ("_082590976" -> "_82590976"), porque el valor es el mismo. El unico
// caso que NO resuelve es que se hayan recortado ceros a la derecha, donde
// "_18259097" significaria 182590970. Comparen un par de nombres reales del
// dataset contra el orden de captura antes de confiar en esto.
std::vector<std::string> ListScansSorted(const std::string& directory);

// Cronometro de pared para instrumentar cada etapa del pipeline.
class Stopwatch {
public:
    void Start() { t0_ = std::chrono::steady_clock::now(); }

    // Milisegundos transcurridos desde Start().
    double ElapsedMs() const {
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0_).count();
    }

private:
    std::chrono::steady_clock::time_point t0_{};
};

}  // namespace recon
