/***
 * @Author: your name
 * @Date: 2022-03-21 20:22:21
 * @LastEditTime: 2022-03-21 20:35:09
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/include/Gun.hpp
 */
#pragma once
#include <iostream>
#include <string>
class Gun {
 private:
  std::string _type;
  int bullet_count;

 public:
  Gun(std::string type) : _type(type), bullet_count(0){};

  void addBullet(int buttle_num);
  bool isShoot();
  ~Gun(){};
};
