/***
 * @Author: your name
 * @Date: 2022-03-21 20:36:16
 * @LastEditTime: 2022-03-21 20:38:21
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/include/solider.hpp
 */
#pragma once

#include <iostream>
#include <string>

#include "Gun.hpp"

class solider {
 private:
  std::string _name;
  Gun* _ptr_gun;

 public:
  solider(std::string name) : _name(name),_ptr_gun(nullptr){};
  void addGun(Gun * ptr_gun);
  void addBulletToGun(int num);
  bool fire();
  ~solider(){};
};
