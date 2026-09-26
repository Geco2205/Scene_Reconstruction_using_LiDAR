#include "recon/RangeFilter.hpp"

#include <cstdint>

#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__)
#define RECON_USE_NEON 1
#include <arm_neon.h>
#else
#define RECON_USE_NEON 0
#endif

namespace recon {

bool NeonEnabled() { return RECON_USE_NEON != 0; }

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
