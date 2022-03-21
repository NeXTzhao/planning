/***
 * @Author: your name
 * @Date: 2022-03-21 20:46:51
 * @LastEditTime: 2022-03-21 20:58:43
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/main.cpp
 */
#include "Gun.hpp"
#include "solider.hpp"
#include "myMath.hpp"

void test() {
  solider sanduo("sanduo");
  sanduo.addGun(new Gun("AK47"));
  sanduo.addBulletToGun(20);
  sanduo.fire();
}

int main() {
  test();
  std::cout << sum(100)<<std::endl;
  return 0;
}
