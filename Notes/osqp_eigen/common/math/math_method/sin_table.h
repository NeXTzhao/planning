/*
 * @Author: wangdezhao
 * @Date: 2022-04-28 18:16:35
 * @LastEditTime: 2022-04-30 00:28:45
 * @FilePath: /osqp_eigen/common/math/qp_spline_2d/include/sin_table.h
 * @Copyright:
 */

/**
 * @file
 * @brief Exports the SIN_TABLE, used by the Angle class.
 */

#pragma once

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

}  // namespace math
}  // namespace common
}  // namespace apollo
