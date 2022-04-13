/*
 * @Author: wangdezhao
 * @Date: 2022-04-08 23:24:17
 * @LastEditTime: 2022-04-12 18:36:47
 * @FilePath: /osqp_eigen/common/math/osqp_interface/include/types.hpp
 * @Copyright:
 */

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

// #include "helper_functions/float_comparisons.hpp"
// #include "common/visibility_control.hpp"

// namespace autoware {
namespace common {
namespace types {
// Aliases to conform to MISRA C++ Rule 3-9-2 (Directive 4.6 in MISRA C).
// Similarly, the stdint typedefs should be used instead of plain int, long etc.
// types. We don't currently require code to comply to MISRA, but we should try
// to where it is easily possible.
using bool8_t = bool;
using char8_t_ = char;
using uchar8_t = unsigned char;
// If we ever compile on a platform where this is not true, float32_t and
// float64_t definitions need to be adjusted.
static_assert(sizeof(float) == 4, "float is assumed to be 32-bit");
using float32_t = float;
static_assert(sizeof(double) == 8, "double is assumed to be 64-bit");
using float64_t = double;

/// pi = tau / 2
constexpr float32_t PI = 3.14159265359F;
/// pi/2
constexpr float32_t PI_2 = 1.5707963267948966F;
/// tau = 2 pi
constexpr float32_t TAU = 6.283185307179586476925286766559F;
// TODO(yunus.caliskan): switch to std::void_t when C++17 is available
/// \brief `std::void_t<> implementation
template <typename... Ts>
using void_t = void;
}  // namespace types
}  // namespace common

