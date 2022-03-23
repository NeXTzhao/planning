/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-23 16:07:39
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /cmake_test_file/math/myMath_test.cpp
 * @Copyright:
 */
#include "myMath.hpp"

#include <gtest/gtest.h>

TEST(testCase, test0) {
  for (double i = 0; i < 100; i++) {
    double temp = (1 + i) * i / 2;
    EXPECT_EQ(sum(i), temp);
  }
  // EXPECT_EQ(sum(100),5050);
}
