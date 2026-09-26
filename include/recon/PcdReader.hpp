// PcdReader.hpp -- lector minimo de PCD (ascii y binary) que extrae solo XYZ.
//
// Los scans del Newer College vienen de un Ouster y traen 9 campos de tamanos
// mixtos (x y z intensity t reflectivity ring ambient range). Por eso el header
// se parsea de forma generica: se leen FIELDS/SIZE/TYPE/COUNT, se calculan los
// offsets de cada campo y se saltan los que no interesan. Asi el mismo lector
// sirve para cualquier PCD con XYZ float32.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace recon {

// Nube en formato SoA (structure of arrays): un arreglo por eje.
// Esto es lo que permite vectorizar el filtro con NEON: los tres arreglos son
// contiguos y alineados, y se pueden cargar de a 4 floats por instruccion.
// Un std::vector<Eigen::Vector3d> (AoS) no permite eso.
struct PointCloudSoA {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;

    std::size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }

    void reserve(std::size_t n) {
        x.reserve(n);
        y.reserve(n);
        z.reserve(n);
    }

    void clear() {
        x.clear();
        y.clear();
        z.clear();
    }

    void push_back(float px, float py, float pz) {
        x.push_back(px);
        y.push_back(py);
        z.push_back(pz);
    }
};

// Lee un .pcd y devuelve solo los puntos XYZ finitos.
//
// Soporta DATA ascii y DATA binary. DATA binary_compressed no esta soportado y
// lanza std::runtime_error con un mensaje claro.
//
// Los puntos con NaN o infinito se descartan durante la lectura: los scans del
// Newer College incluyen retornos invalidos y si llegan al ICP lo rompen.
//
// Lanza std::runtime_error si el archivo no abre, si el header esta malformado
// o si no encuentra los campos x, y, z como float32.
PointCloudSoA ReadPcd(const std::string& path);

}  // namespace recon
