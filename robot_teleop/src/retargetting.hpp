#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace retarget
{
    using Vec3 = std::array<double, 3>;

    inline const std::array<std::array<int, 5>, 5> kChains ={{
        {0, 1, 2, 3, 4},
        {0, 5, 6, 7, 8},
        {0, 9, 10, 11, 12},
        {0, 13, 14, 15, 16},
        {0, 17, 18, 19, 20}
    }};

    // vector functions
    inline Vec3 point(const double *d, int i) { return {d[3 * i], d[3 * i + 2], d[3 * i+2]};}
    inline Vec3 sub(const Vec3 &a, const Vec3 &b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
    inline double dot(const Vec3 &a, const Vec3 &b) {return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}
    inline double norm(const Vec3 &v) { return std::sqrt(dot(v, v));}
    inline double angle(const Vec3 &u, const Vec3 &v)
    {
        const double nu = norm(u), nv = norm(v);
        if (nu < 1e-9 || nv < 1e-9) return 0.0;
        return std::acos(std::clamp(dot(u, v) / (nu * nv), -1.0, 1.0));       
    } 

    inline std::array<double, 3> chainAngles(const double *landmarks, const std::array<int, 5> &ch){
        const Vec3 b0 = sub(point(landmarks, ch[1]), point(landmarks, ch[0]));
        const Vec3 b1 = sub(point(landmarks, ch[2]), point(landmarks, ch[1]));
        const Vec3 b2 = sub(point(landmarks, ch[3]), point(landmarks, ch[2]));
        const Vec3 b3 = sub(point(landmarks, ch[4]), point(landmarks, ch[3]));
        return {angle(b0, b1), angle(b1, b2), angle(b2, b3)};
    }
    inline double normalizeCurl(double raw, double open, double closed){
        const double span = closed - open;
        if (std::abs(span) < 1e-6) return 0.0;
        return std::clamp((raw - open) / span, 0.0, 1.0);
    }

        // 1st order low pass filter w independent time const
    inline double smoothStep(double prev, double target, double dt, double tau){
        if (tau <= 0.0) return target;
        const double a = 1.0 - std::exp(-dt / tau);
        return prev + a * (target - prev);
    }
}