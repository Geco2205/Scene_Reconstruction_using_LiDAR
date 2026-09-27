#include "recon/RangeFilter.hpp"

#include <cstdint>

// RECON_NEON_REQUESTED viene de la opcion RECON_OPT_NEON de CMake (ON por
// defecto). Apagarla en la Kria/Jetson permite medir el efecto de NEON con el
// mismo codigo: queda solo el camino escalar.
#ifndef RECON_NEON_REQUESTED
#define RECON_NEON_REQUESTED 1
#endif

#if RECON_NEON_REQUESTED && (defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__))
#define RECON_USE_NEON 1
#include <arm_neon.h>
#else
#define RECON_USE_NEON 0
#endif

namespace recon {

bool NeonEnabled() { return RECON_USE_NEON != 0; }

#if RECON_SOA_ALIGNED
// ---------------------------------------------------------------------------
// Variante con SoA alineado (RECON_OPT_SOA_ALIGNED=ON)
//
// 1. Los arreglos de entrada y salida empiezan en una frontera de 64 B, lo que
//    se le promete al compilador con __builtin_assume_aligned. Las cargas NEON
//    del bloque principal nunca cruzan una linea de cache.
// 2. La salida se reserva una vez con resize(n) (sin inicializar, ver
//    AlignedAllocator) y se escribe por indice. El indice avanza segun la
//    mascara, sin ramas: se evitan el chequeo de capacidad de push_back y los
//    saltos mal predichos cuando el filtro deja pasar o no un punto.
// ---------------------------------------------------------------------------
void FilterByRange(const PointCloudSoA& in, float r_min, float r_max, PointCloudSoA& out) {
    const std::size_t n = in.size();
    out.resize(n);

    const float min2 = r_min * r_min;
    const float max2 = r_max * r_max;

    const float* __restrict px =
        static_cast<const float*>(__builtin_assume_aligned(in.x.data(), kSoaAlignment));
    const float* __restrict py =
        static_cast<const float*>(__builtin_assume_aligned(in.y.data(), kSoaAlignment));
    const float* __restrict pz =
        static_cast<const float*>(__builtin_assume_aligned(in.z.data(), kSoaAlignment));
    float* __restrict ox = out.x.data();
    float* __restrict oy = out.y.data();
    float* __restrict oz = out.z.data();

    std::size_t i = 0;
    std::size_t k = 0;

#if RECON_USE_NEON
    const float32x4_t vmin2 = vdupq_n_f32(min2);
    const float32x4_t vmax2 = vdupq_n_f32(max2);

    for (; i + 4 <= n; i += 4) {
        const float32x4_t vx = vld1q_f32(px + i);
        const float32x4_t vy = vld1q_f32(py + i);
        const float32x4_t vz = vld1q_f32(pz + i);

        float32x4_t d2 = vmulq_f32(vx, vx);
        d2 = vmlaq_f32(d2, vy, vy);
        d2 = vmlaq_f32(d2, vz, vz);
        const uint32x4_t keep = vandq_u32(vcgeq_f32(d2, vmin2), vcleq_f32(d2, vmax2));

        // Compactacion sin ramas: cada lane se escribe en la posicion k y k
        // avanza 1 solo si el lane paso. La mascara vale 0 o 0xFFFFFFFF.
        uint32_t mask[4];
        vst1q_u32(mask, keep);
        for (int j = 0; j < 4; ++j) {
            ox[k] = px[i + j];
            oy[k] = py[i + j];
            oz[k] = pz[i + j];
            k += mask[j] & 1u;
        }
    }
#endif

    // Cola escalar (y camino completo en x86), tambien sin ramas.
    for (; i < n; ++i) {
        const float d2 = px[i] * px[i] + py[i] * py[i] + pz[i] * pz[i];
        ox[k] = px[i];
        oy[k] = py[i];
        oz[k] = pz[i];
        k += static_cast<std::size_t>((d2 >= min2) & (d2 <= max2));
    }

    out.resize(k);
}

#else
// ---------------------------------------------------------------------------
// Variante base (RECON_OPT_SOA_ALIGNED=OFF): la original del prototipo.
// ---------------------------------------------------------------------------
void FilterByRange(const PointCloudSoA& in, float r_min, float r_max, PointCloudSoA& out) {
    out.clear();
    out.reserve(in.size());

    const float min2 = r_min * r_min;
    const float max2 = r_max * r_max;

    const float* __restrict px = in.x.data();
    const float* __restrict py = in.y.data();
    const float* __restrict pz = in.z.data();
    const std::size_t n = in.size();
    std::size_t i = 0;

#if RECON_USE_NEON
    const float32x4_t vmin2 = vdupq_n_f32(min2);
    const float32x4_t vmax2 = vdupq_n_f32(max2);

    for (; i + 4 <= n; i += 4) {
        const float32x4_t vx = vld1q_f32(px + i);
        const float32x4_t vy = vld1q_f32(py + i);
        const float32x4_t vz = vld1q_f32(pz + i);

        // d2 = x*x + y*y + z*z  (dos multiply-accumulate encadenados)
        float32x4_t d2 = vmulq_f32(vx, vx);
        d2 = vmlaq_f32(d2, vy, vy);
        d2 = vmlaq_f32(d2, vz, vz);

        // keep = (d2 >= min2) && (d2 <= max2)
        const uint32x4_t keep = vandq_u32(vcgeq_f32(d2, vmin2), vcleq_f32(d2, vmax2));

        // Atajo: si ningun lane pasa, no se toca la salida.
        // vmaxvq_u32 solo existe en AArch64; en ARMv7 se omite el atajo.
#if defined(__aarch64__)
        if (vmaxvq_u32(keep) == 0) continue;
#endif
        uint32_t mask[4];
        vst1q_u32(mask, keep);
        for (int k = 0; k < 4; ++k) {
            if (mask[k]) out.push_back(px[i + k], py[i + k], pz[i + k]);
        }
    }
#endif

    // Cola escalar (y camino completo en x86).
    for (; i < n; ++i) {
        const float d2 = px[i] * px[i] + py[i] * py[i] + pz[i] * pz[i];
        if (d2 >= min2 && d2 <= max2) out.push_back(px[i], py[i], pz[i]);
    }
}
#endif

std::vector<Eigen::Vector3d> ToEigen(const PointCloudSoA& cloud) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        points.emplace_back(static_cast<double>(cloud.x[i]),
                            static_cast<double>(cloud.y[i]),
                            static_cast<double>(cloud.z[i]));
    }
    return points;
}

}  // namespace recon
