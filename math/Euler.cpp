/**
 * @file Euler.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-10-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <stdio.h>
#include <iostream>
#include <string>
using namespace std;
/**
 * @brief 利用欧拉法解微分方程
 *
 */

int main() {
  double times = 1024;
  double x = 1, y = 1, dy_dx;
  for (double x = 1; x <= 2; x += 1.0 / times) {
    dy_dx = x * y;
    // 计算y的变化量
    y += dy_dx / times;
  }
  printf("y=%f", y);
  return 0;
}