/***
 * @Author: your name
 * @Date: 2022-03-21 20:46:51
 * @LastEditTime: 2022-03-21 20:58:43
 * @LastEditors: your name
 * @Description:
 * @FilePath: /cmake_test_file/main.cpp
 */

#include "config.h"
#include "Gun.hpp"
#include "solider.hpp"

#ifdef USE_MATH
  #include "myMath.hpp"
#endif

void test() {
  solider sanduo("sanduo");
  sanduo.addGun(new Gun("AK47"));
  sanduo.addBulletToGun(20);
  sanduo.fire();
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    // report version
    std::cout << argv[0] << " Version " << CMAKE_TEST_VERSION_MAJOR << " . "
              << CMAKE_TEST_VERSION_MINOR << std::endl;
  }

  test();
  
  #ifdef USE_MATH
    std::cout << sum(100) << std::endl;
  #else
    std::cout << "not use math" << std::endl;
  #endif

  return 0;
}
