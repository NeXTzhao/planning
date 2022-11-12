/*
 * @Author: wangdezhao
 * @Date: 2022-04-09 00:16:20
 * @LastEditTime: 2022-04-10 22:09:40
 * @FilePath: /osqp_eigen/src/csc_matrix_osqp.cpp
 * @Copyright:
 */
#include "csc_matrix_osqp.hpp"

auto check_result =
    [](const std::tuple<std::vector<float64_t>, std::vector<float64_t>, int,
                        int>& result) {
      std::cout << std::get<2>(result) << '\n'
                << std::get<3>(result) << '\n'
                << std::get<0>(result).size() << '\n'
                << std::get<1>(result).size() << '\n'
                << std::get<0>(result)[0] << '\n'
                << std::get<0>(result)[1] << '\n'
                << std::get<1>(result)[0] << '\n'
                << std::get<1>(result)[1] << '\n'
                << std::endl;
    };

int main() {
  //CSC 
  Eigen::MatrixXd rect2(4, 4);
  rect2 << 1.0, 7.0, 0.0, 0.0, 
           0.0, 2.0, 8.0, 0.0, 
           5.0, 0.0, 3.0, 9.0, 
           0.0, 6.0, 0.0, 4.0;

  const CSC_Matrix rect_m2 = calCSCMatrix(rect2);
  printCSCMatrix(rect_m2);
  

  //osqp
  common::osqp::OSQPInterface osqp;
  Eigen::MatrixXd P(2, 2);
  P << 4, 1, 1, 2;
  Eigen::MatrixXd A(2, 4);
  A << 1, 1, 1, 0, 0, 1, 0, 1;
  std::vector<float64_t> q = {1.0, 1.0};
  std::vector<float64_t> l = {1.0, 0.0, 0.0, -common::osqp::INF};
  std::vector<float64_t> u = {1.0, 0.7, 0.7, common::osqp::INF};
  std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int> result =
      osqp.optimize(P, A, q, l, u);
  // check_result(result);

  return 0;
}