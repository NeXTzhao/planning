/*
 * @Author: your name
 * @Date: 2021-10-13 22:22:37
 * @LastEditTime: 2021-10-15 19:54:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ros/home/next/ros_workspace/routing_planning/math/eigen.cpp
 */
#include <stdio.h>
#include <Eigen/Eigen>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace Eigen;

using namespace std;

// Eigen::Matrix<float, 1, 4> cal_dlqr(Eigen::Matrix4f A, Eigen::Vector4f B,
// Eigen::Matrix4f Q,
//               Eigen::MatrixXf R) {
//   // 设置最大循环迭代次数
//   int numLoop = 500;
//   // 设置目标极小值
//   float minValue = 0.01;
//   Eigen::Matrix4f p_old;
//   Eigen::Matrix4f p_new;
//   for (int i = 0; i < numLoop; i++) {
//     p_old = Q;
//     // B.inverse()求逆
//     p_new = Q + A.transpose() * p_old * A -
//             A.transpose() * p_old * B *
//                 ((R + B.transpose() * p_old * B).inverse()) * B.transpose() *
//                 p_old * A;
//     // p.determinant()求行列式
//     if (std::abs((p_old - p_new).determinant()) <= minValue) {
//       break;
//     } else {
//       p_old = p_new;
//     }
//   }
//   // Eigen::RowVector4cf k;

//   Eigen::Matrix<float, 1, 4> k;
//    k =
//       (R + B.transpose() * p_new * B).inverse() * B.transpose() * p_new * A;
//   // return k;
// }

int main() {
  /*单位矩阵**/
  //   Matrix<double, Dynamic, Dynamic> m_matrix;
  // //   MatrixXd m_matrix2(3, 3);
  // Eigen::Matrix4f A;
  // // m_matrix2 << 1,2,3,
  // // 	4,5,6,
  // // 	7,8,8;
  // // cout << "MatrixXd::Identity(5, 4):\n"<<MatrixXd::Identity(5, 4) <<
  // // endl;;
  // //   m_matrix.setIdentity(4, 4);
  // A.setIdentity(4, 4);

  // cout << "a.setIdentity(4, 4):\n" << A << endl;

  // // /*求逆矩阵需要先判断是否可逆**/

  // // cout << "m_matrix2.inverse（）:\n" << m_matrix2.inverse() << endl;

  // // /*逐元素取倒数**/
  // // cout << "m_matrix.array().inverse():\n" << m_matrix.array().inverse() <<
  // // endl;
  // // cout << "m_matrix.cwiseInverse():\n" << m_matrix.cwiseInverse() << endl;

  // Eigen::Matrix2d c;
  // c << 1, 2, 3, -5;
  // cout << "c: \n" << c << endl;
  // wiseAbs()求绝对值、maxcoff()求最大系数
  // cout << c.cwiseAbs().maxCoeff() << endl;
  // cout << std::abs(c.determinant()) << endl;

  // Eigen::Matrix2d d;
  // d << 1, 0, 1, 3;
  // //转置、伴随
  // // std::cout << c << std::endl << std::endl;
  // // std::cout << "转置\n" << c.transpose() << std::endl << std::endl;
  // // std::cout << "伴随\n" << c.adjoint() << std::endl << std::endl;
  // //逆矩阵、行列式
  // std::cout << "行列式： " << (c - d).determinant() << std::endl;
  // // std::cout << "逆矩阵\n" << c.inverse() << std::endl;

  // // Eigen::RowVector4cf k;
  // Eigen::Matrix<float, 4, 1> k;
  // k << 1, 2, 3, 4;
  // cout << "k(4, 4):\n" << k(3) << endl;

  // float dis_min = std::numeric_limits<float>::max();
  // float dis_max = std::numeric_limits<float>::min();

  // dis_max = 1 / 0;
  // cout << dis_min << "\n" << dis_max << endl;

  // Eigen::Matrix<float, 1, 4> k;
  // k << 0, -31.6239, 89.108, 55.2798;
  // Eigen::Matrix<float, 4, 1> q;
  // q << -0.468715, -0.941237, -0.33404, -0.251617;
  // float a = -k * q - 15.919;
  // cout << a * M_PI / 180 << endl;
  // int rows = 5, cols = 5;
  // // MatrixXf m(rows, cols);
  // // m << (Matrix3f() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished(),
  // //     MatrixXf::Zero(3, cols - 3), MatrixXf::Zero(rows - 3, 3),
  // //     MatrixXf::Identity(rows - 3, cols - 3);
  // // cout << m;

  // cout << MatrixXf::Zero(3, cols - 3) << endl;

  Eigen::Matrix3d matrix_33;  //初始化为0
  ////Matrix3d实质上是Eigen::Matrix<double, 3, 3>
  matrix_33 << 1, 2, 3, 4, 5, -6, 7, 8, 9;

  //   cout << matrix_33.transpose() << endl;    //转置
  // cout << matrix_33.sum() << endl;          //各元素和
  // cout << matrix_33.trace() << endl;         //迹
  // cout << matrix_33 * 10 << endl;            //数乘
  // cout << matrix_33.inverse() << endl;        //逆
  // cout << matrix_33.determinant() << endl;     //行列式
  // cout << matrix_33.cwiseAbs() << endl;  //绝对值
  cout << matrix_33.cwiseAbs().maxCoeff() << endl;  //求最大系数
  return 0;
}
