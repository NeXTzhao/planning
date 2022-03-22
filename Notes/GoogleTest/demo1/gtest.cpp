/*** 
 * @Author: your name
 * @Date: 2022-03-22 16:54:28
 * @LastEditTime: 2022-03-22 17:41:48
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /cmake_test_file/gtest.cpp
 */

#include <gtest/gtest.h>

int add(int a ,int b){
  return a+b;
}

TEST(testCase , test0){
  EXPECT_EQ(add(8,11),19);
}

TEST(testCase1 , test0){
  EXPECT_EQ(add(7,11),18);
}

int main(){
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}