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
  // float cd = 0.0;
  // float cv = 0.0;
  // float cf = 0.0;

  Vec_f t;
  Vec_f x;
  Vec_f x_d;
  Vec_f x_dd;
  // Vec_f d_ddd;
  Vec_f y;
  Vec_f y_d;
  Vec_f y_dd;
  // Vec_f s_ddd;

  // Vec_f x;
  // Vec_f y;
  // Vec_f yaw;
  // Vec_f ds;
  // Vec_f c;

  Vec_f k;  // curvature
  Vec_f threat;

  // float max_speed;
  // float max_accel;
  // float max_curvature;
};

using Vec_Path = std::vector<FrenetPath>;
}
#endif
