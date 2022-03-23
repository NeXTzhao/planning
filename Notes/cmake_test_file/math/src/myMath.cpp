/*** 
 * @Author: your name
 * @Date: 2022-03-21 21:50:09
 * @LastEditTime: 2022-03-21 21:52:46
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /cmake_test_file/math/myMath.cpp
 */
#include "myMath.hpp"

double sum(int num) {
  if (num < 2) {
    return num;
  }

  double num1 = (1 + num) * num / 2;

  return num1;
}