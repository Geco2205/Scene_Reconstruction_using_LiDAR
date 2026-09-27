// PcdReader.hpp -- lector minimo de PCD (ascii y binary) que extrae solo XYZ.
//
// Los scans del Newer College vienen de un Ouster y traen 9 campos de tamanos
// mixtos (x y z intensity t reflectivity ring ambient range). Por eso el header
// se parsea de forma generica: se leen FIELDS/SIZE/TYPE/COUNT, se calculan los
// offsets de cada campo y se saltan los que no interesan. Asi el mismo lector
// sirve para cualquier PCD con XYZ float32.

#pragma once

#include <cstddef>
#include <new>
#include <string>
#include <utility>
#include <vector>

#ifndef RECON_SOA_ALIGNED
#define RECON_SOA_ALIGNED 0
#endif

namespace recon {

// Alineamiento de los arreglos SoA con RECON_OPT_SOA_ALIGNED. 64 B es una linea
// de cache en x86 y en los Cortex-A53/A57, y cubre AVX-512 (64 B), AVX (32 B)
// y NEON (16 B). Ver Cap. 2, "Alineamiento y programacion orientada en cache".
constexpr std::size_t kSoaAlignment = 64;

// Asignador alineado. Ademas, construct() sin argumentos NO inicializa el valor
// (default-init en vez de value-init): asi resize(n) reserva sin escribir n
// ceros que igual se van a sobrescribir. Solo tiene sentido para tipos triviales.
template <class T, std::size_t Align>
struct AlignedAllocator {
    using value_type = T;
    template <class U>
    struct rebind { using other = AlignedAllocator<U, Align>; };

    AlignedAllocator() noexcept = default;
    template <class U>
    AlignedAllocator(const AlignedAllocator<U, Align>&) noexcept {}

    T* allocate(std::size_t n) {
        return static_cast<T*>(::operator new(n * sizeof(T), std::align_val_t{Align}));
    }
    void deallocate(T* p, std::size_t) noexcept {
        ::operator delete(p, std::align_val_t{Align});
    }

    template <class U>
    void construct(U* p) noexcept { ::new (static_cast<void*>(p)) U; }
    template <class U, class... Args>
    void construct(U* p, Args&&... args) {
        ::new (static_cast<void*>(p)) U(std::forward<Args>(args)...);
    }

    template <class U>
    bool operator==(const AlignedAllocator<U, Align>&) const noexcept { return true; }
    template <class U>
    bool operator!=(const AlignedAllocator<U, Align>&) const noexcept { return false; }
};

#if RECON_SOA_ALIGNED
using FloatArray = std::vector<float, AlignedAllocator<float, kSoaAlignment>>;
#else
using FloatArray = std::vector<float>;
#endif

// Nube en formato SoA (structure of arrays): un arreglo por eje.
// Esto es lo que permite vectorizar el filtro con NEON: los tres arreglos son
// contiguos y se pueden cargar de a 4 floats por instruccion. Un
// std::vector<Eigen::Vector3d> (AoS) no permite eso.
//
// Con RECON_OPT_SOA_ALIGNED los tres arreglos ademas empiezan en una frontera
// de 64 B, y el lector y el filtro escriben por indice (resize + shrink) en vez
// de push_back, que revisa la capacidad en cada punto.
struct PointCloudSoA {
    FloatArray x;
    FloatArray y;
    FloatArray z;

    std::size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }

    void reserve(std::size_t n) {
        x.reserve(n);
        y.reserve(n);
        z.reserve(n);
    }

    void resize(std::size_t n) {
        x.resize(n);
        y.resize(n);
        z.resize(n);
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
