/*
 * @Author: your name
 * @Date: 2021-10-13 10:43:57
 * @LastEditTime: 2021-10-13 17:33:19
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /quintic_lqr/include/frenet_path.h
 */
#ifndef _FRENET_PATH_H
#define _FRENET_PATH_H

#include <array>
#include <iostream>
#include <string>
#include <vector>
#include "cpprobotics_types.h"

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

  Vec_f k;  // curvature
  Vec_f threat;
};

using Vec_Path = std::vector<FrenetPath>;
}
#endif
