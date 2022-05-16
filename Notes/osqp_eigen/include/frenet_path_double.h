/**
 * @file frenet_path_double.h
 * @brief 
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-02 10:16:55
 * 
 * @copyright Copyright (c) 2022 
 */

#pragma once

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include "cpprobotics_types_double.h"

namespace cpprobotics {

class FrenetPath {
 public:
  Vec_f t;
  Vec_f x;
  Vec_f x_d;
  Vec_f x_dd;

  Vec_f y;
  Vec_f y_d;
  Vec_f y_dd;

  Vec_f kappa;  // curvature
  Vec_f theta;
  Vec_f s;

  Vec_f dkappa;
};

using Vec_Path = std::vector<FrenetPath>;
}
