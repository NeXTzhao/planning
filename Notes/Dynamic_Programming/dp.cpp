// /**
//  * @file dp.cpp
//  * @brief 将DR_CAN讲的动态规划程序(m语言)转成c++代码
//  * @author Wang Dezhao (1282507109@qq.com)
//  * @version 1.0
//  * @date 2023-06-14 17:25:18
//  *
//  * @copyright Copyright (c) 2023
//  */

#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

const double INFINITY_VAL = std::numeric_limits<double>::infinity();

bool compareCost(const std::tuple<double, double, double>& tuple1,
                 const std::tuple<double, double, double>& tuple2) {
  return std::get<0>(tuple1) < std::get<0>(tuple2);
}

class DynamicProgramming {
 public:
  DynamicProgramming()
      : N_h(5),
        N_v(5),
        u_min(-3),
        u_max(2),
        h_min(0),
        h_max(10),
        v_min(0),
        v_max(5),
        h_final(10),
        v_final(0) {
    Hd.resize(N_h);
    Vd.resize(N_v);
    v_res.resize(N_h);
    J_costtogo.resize(N_h, std::vector<std::tuple<double, double, double>>(
                               N_v, std::make_tuple(0.0, 0.0, 0.0)));
    Input_acc.resize(N_h);
  }

  void computeOptimalControl() {
    initializeStates();
    computeCostToGoMatrix();
  }

  void plotResults() {
    plt::subplot(2, 1, 1);
    plt::named_plot("v_h", v_res, Hd, "r.");
    plt::xlabel("v");
    plt::ylabel("h");
    plt::legend();

    plt::subplot(2, 1, 2);
    std::vector<double> acc_all;
    for (int k = 0; k < N_h; ++k) {
      acc_all.push_back(Input_acc[k]);
    }
    plt::named_plot("acc_h", acc_all, Hd, "b.");
    plt::xlabel("acc");
    plt::ylabel("h");
    plt::legend();
    plt::show();
  }

 private:
  int N_h;
  int N_v;
  double u_min;
  double u_max;
  double h_min;
  double h_max;
  double v_min;
  double v_max;
  double h_final;
  double v_final;

  std::vector<double> Hd;
  std::vector<double> Vd;
  std::vector<double> v_res;
  std::vector<std::vector<std::tuple<double, double, double>>> J_costtogo;
  std::vector<double> Input_acc;

  void initializeStates() {
    for (int i = 0; i < N_h; ++i) {
      Hd[i] = h_min + (h_max - h_min) / N_h * i;
    }

    for (int i = 0; i < N_v; ++i) {
      // Vd[i] = v_min + (v_max - v_min) / N_v * i;
      Vd[i] = i;
    }
  }

  void computeCostToGoMatrix() {
    std::vector<double> T_delta(N_v);
    std::vector<double> acc(N_v);
    std::vector<double> J_temp(N_v);
    double min_val = INFINITY_VAL;
    int min_index = 0;
    double cost_temp = 0.0;
    double acc_temp = 0.0;

    for (int k = 1; k < N_h; ++k) {
      for (int i = 0; i < N_v; ++i) {
        cost_temp = INFINITY_VAL;
        for (int j = 0; j < N_v; ++j) {
          double v_avg = 0.5 * (std::get<1>(J_costtogo[k - 1][j]) + Vd[i]);
          auto T_delta = (h_max - h_min) / (N_h * v_avg);
          auto J_temp = T_delta;
          acc[j] = (std::get<1>(J_costtogo[k - 1][j]) - Vd[i]) / T_delta;

          if (acc[j] < u_min || acc[j] > u_max) {
            J_temp = INFINITY_VAL;
          }
          J_temp += std::get<0>(J_costtogo[k - 1][j]);

          if (J_temp < min_val) {
            cost_temp = J_temp;
            min_index = j;
            acc_temp = acc[j];
          }
        }
        J_costtogo[k][i] = std::make_tuple(cost_temp, Vd[i], acc_temp);
      }
      // 使用std::min_element和自定义的比较函数找到最小的cost值的迭代器
      auto min_cost_iter = std::min_element(J_costtogo[k].begin(),
                                            J_costtogo[k].end(), compareCost);

      // 通过解引用迭代器获取最小的cost值对应的v和a
      double min_v = std::get<1>(*min_cost_iter);
      double min_a = std::get<2>(*min_cost_iter);

      v_res[k] = min_v;
      Input_acc[k] = min_a;
    }

    // 打印J_costtogo
    std::cout << "J_costtogo:" << std::endl;

    for (int i = 0; i < N_h; ++i) {
      for (int j = 0; j < N_v; ++j) {
        double val1 = std::get<0>(J_costtogo[i][j]);
        double val3 = std::get<2>(J_costtogo[i][j]);

        std::cout << std::fixed << std::setprecision(1) << "(" << val1 << " , "
                  << val3 << ")"
                  << "\t";
      }
      std::cout << std::endl;
    }
  }
};

int main() {
  DynamicProgramming dp;
  dp.computeOptimalControl();
  dp.plotResults();
  return 0;
}
