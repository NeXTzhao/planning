/***
 * @Author: your name
 * @Date: 2022-03-21 20:36:27
 * @LastEditTime: 2022-03-21 20:58:54
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/src/solider.cpp
 */
#include "solider.hpp"

void solider::addBulletToGun(int num) { this->_ptr_gun->addBullet(num); }

void solider::addGun(Gun* ptr_gun) { this->_ptr_gun = ptr_gun; }

bool solider::fire() {
  std::cout << "fire is ok" << std::endl;
  return this->_ptr_gun->isShoot();
}
