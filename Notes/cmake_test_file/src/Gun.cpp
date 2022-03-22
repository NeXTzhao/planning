/***
 * @Author: your name
 * @Date: 2022-03-21 20:22:05
 * @LastEditTime: 2022-03-21 20:35:13
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/src/Gun.cpp
 */
#include "Gun.hpp"

void Gun::addBullet(int buttle_num) { this->bullet_count += bullet_count; }

bool Gun::isShoot() {
  if (this->bullet_count <= 0) {
    std::cout << "there is no buttle" << std::endl;
    return false;
  }
  this->bullet_count -= 1;
  return true;
}