// RangeFilter.hpp -- filtro por rango min/max y conversion SoA -> Eigen.
//
// Esta es la etapa que se vectoriza con NEON. Se compara contra el rango al
// cuadrado para evitar la raiz cuadrada por punto.
//
// Nota sobre la vectorizacion: el calculo de x^2+y^2+z^2 y la comparacion se
// hacen de a 4 puntos con NEON, pero la escritura del resultado es escalar.
// Eso es inevitable porque la salida es de tamano variable (compactacion) y
// ARMv8 NEON no tiene una instruccion de compress-store. Aun asi la aritmetica,
// que es la parte cara, si se vectoriza.

#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "recon/PcdReader.hpp"

namespace recon {

// Devuelve true si la build actual esta usando el camino NEON.
// Sirve para reportarlo en el log de perfilado.
bool NeonEnabled();

// Conserva los puntos cuya norma esta en [r_min, r_max].
// 'out' se limpia antes de escribir. 'in' y 'out' no pueden ser el mismo objeto.
void FilterByRange(const PointCloudSoA& in, float r_min, float r_max, PointCloudSoA& out);

// Convierte SoA (float32) al formato que consume KISS-ICP (AoS, double).
// La conversion de precision ocurre aqui: el PCD guarda float32 y Eigen::Vector3d
// es double. No se pierde nada porque float32 cabe exacto en double.
std::vector<Eigen::Vector3d> ToEigen(const PointCloudSoA& cloud);

}  // namespace recon
