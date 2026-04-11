/*******************************************************************************

Purpose:
  General math utilities for GN&C models (C++11 compatible)

Library dependencies:
  (())

*******************************************************************************/

#pragma once

#include <cmath>

namespace gnc {
namespace math {

//--------------------------------------------------
// Clamp (C++11 compatible)
//--------------------------------------------------
template <typename T>
inline T clamp(const T& val, const T& min_val, const T& max_val)
{
    return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

//--------------------------------------------------
// Sign function
//--------------------------------------------------
template <typename T>
inline T sign(const T& val)
{
    return (T(0) < val) - (val < T(0));
}

//--------------------------------------------------
// Deadband
//--------------------------------------------------
template <typename T>
inline T deadband(const T& val, const T& threshold)
{
    if (std::abs(val) < threshold) {
        return T(0);
    }
    return val;
}

//--------------------------------------------------
// Safe normalize (3-vector)
//--------------------------------------------------
inline void normalize3(const double in[3], double out[3])
{
    double norm = std::sqrt(in[0]*in[0] + in[1]*in[1] + in[2]*in[2]);

    if (norm > 1e-12) {
        out[0] = in[0] / norm;
        out[1] = in[1] / norm;
        out[2] = in[2] / norm;
    } else {
        out[0] = out[1] = out[2] = 0.0;
    }
}

//--------------------------------------------------
// Cross product
//--------------------------------------------------
inline void cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

//--------------------------------------------------
// Dot product
//--------------------------------------------------
inline double dot3(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

} // namespace math
} // namespace gnc