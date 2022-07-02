/**
 * @file cartesianFrenet.hpp
 * @brief
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-02 10:53:51
 *
 * @copyright Copyright (c) 2022
 */

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

#include "cartesian_frenet_conversion.h"
#include "cpprobotics_types_double.h"
#include "matplotlibcpp.h"
#include "quinticPolynomial.hpp"

using namespace apollo;
using namespace common;
using namespace math;
using namespace cpprobotics;
using Poi_d = std::array<double, 2>;

namespace plt = matplotlibcpp;
