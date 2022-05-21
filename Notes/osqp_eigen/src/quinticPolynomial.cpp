#include "quinticPolynomial.hpp"

namespace cpprobotics {
QuinticPolynomial::QuinticPolynomial(PointState x_start, PointState x_end,
                                     PointState y_start, PointState y_end,
                                     double TotalTimes)
    : x_start_(x_start),
      x_end_(x_end),
      y_start_(y_start),
      y_end_(y_end),
      TotalTimes_(TotalTimes),
      a0(x_start.xy),
      a1(x_start.v),
      a2(x_start.a),
      b0(y_start.xy),
      b1(y_start.v),
      b2(y_start.a) {}

void QuinticPolynomial::cal_longiPoly_coff() {
  double xe = x_end_.xy;
  double vxe = x_end_.v;
  double axe = x_end_.a;

  double T = TotalTimes_;
  // 起始和终点的矩阵方程表达式
  Eigen::Matrix3d A;
  // std::pow(x, y)计算x的y次幂
  A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
      4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
      20 * std::pow(T, 3);
  Eigen::Vector3d B;
  B << xe - a0 - a1 * T - a2 * std::pow(T, 2), vxe - a1 - 2 * a2 * T,
      axe - 2 * a2;
  // Eigen矩阵colPivHouseholderQr().solve()求解Ax=b
  Eigen::Vector3d c_eigen = A.colPivHouseholderQr().solve(B);
  a3 = c_eigen[0];
  a4 = c_eigen[1];
  a5 = c_eigen[2];
}

void QuinticPolynomial::cal_latPoly_coff() {
  double ye = y_end_.xy;
  double vye = y_end_.v;
  double aye = y_end_.a;

  double T = TotalTimes_;
  // 起始和终点的矩阵方程表达式
  Eigen::Matrix3d C;
  // std::pow(x, y)计算x的y次幂
  C << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2),
      4 * std::pow(T, 3), 5 * std::pow(T, 4), 6 * T, 12 * std::pow(T, 2),
      20 * std::pow(T, 3);
  Eigen::Vector3d D;
  D << ye - b0 - b1 * T - b2 * std::pow(T, 2), vye - b1 - 2 * b2 * T,
      aye - 2 * b2;
  // Eigen矩阵colPivHouseholderQr().solve()求解Ax=b
  Eigen::Vector3d b_eigen = C.colPivHouseholderQr().solve(D);

  b3 = b_eigen[0];
  b4 = b_eigen[1];
  b5 = b_eigen[2];

  // std::cout <<"b3,b4,b5:"<<b3<<","<<b4<<","<<b5<<'\n';
}

void QuinticPolynomial::getPloyPath(FrenetPath& fp, const double DT) {
  cal_longiPoly_coff();
  cal_latPoly_coff();
  for (double t = 0; t < TotalTimes_; t += DT) {
    double x = calc_point_x(t);
    // std::cout << ":x:" << x << '\n';

    double xd = calc_point_xd(t);
    double xdd = calc_point_xdd(t);
    double xddd = calc_point_xddd(t);

    fp.t.push_back(t);
    fp.x.push_back(x);
    fp.x_d.push_back(xd);
    fp.x_dd.push_back(xdd);

    double y_x_t = calc_point_y_x(x);
    // std::cout << ":y:" << y_x_t << '\n';
    // std::cout << '\n';
    double y_x_d = calc_point_y_x_d(x);

    double y_x_t_d = calc_point_y_t_d(y_x_d, xd);

    double y_x_dd = calc_point_y_x_dd(x);
    double y_x_t_dd = calc_point_y_t_dd(y_x_dd, xd, y_x_d, xdd);

    double y_x_ddd = calc_point_y_x_ddd(x);
    double y_x_t_ddd =
        calc_point_y_t_ddd(y_x_ddd, y_x_dd, y_x_d, xddd, xdd, xd);
    fp.y.push_back(y_x_t);
    // if (t > 0) {
    //   fp.y.push_back(y_x_t);
    // } else {
    //   fp.y.push_back(x);
    // }
    fp.y_d.push_back(y_x_t_d);
    fp.y_dd.push_back(y_x_t_dd);
    fp.kappa.push_back(calc_point_k(y_x_dd, y_x_d));
    fp.dkappa.push_back(
        cal_point_k_derivative(xd, xdd, xddd, y_x_t_d, y_x_t_dd, y_x_t_ddd));
  }

  double delta_s = 0;
  for (size_t i = 0; i < fp.t.size() - 1; ++i) {
    double dy = fp.y[i + 1] - fp.y[i];
    double dx = fp.x[i + 1] - fp.x[i];
    fp.theta.push_back(calc_point_theta(dy, dx));

    double delta_y_t = fp.y.at(i + 1) - fp.y.at(i);
    double delta_x_t = fp.x.at(i + 1) - fp.x.at(i);
    delta_s += calc_ploynomial_s(delta_y_t, delta_x_t);
    fp.s.push_back(delta_s);
  }
}

void QuinticPolynomial::matchPoint(const FrenetPath& fp,
                                   const double current_post_x,
                                   const double current_post_y, int pre_index,
                                   int& index) {
  //计算上一个的匹配点的位矢
  std::array<double, 2> pre_cur_error{current_post_x - fp.x.at(pre_index),
                                      current_post_y - fp.y.at(pre_index)};
  double heading = fp.theta.at(pre_index);
  std::array<double, 2> pre_heading{cos(heading), sin(heading)};

  double innerProd = pre_cur_error.at(0) * pre_heading.at(0) +
                     pre_cur_error.at(1) * pre_heading.at(1);
  // TODO: 车往前或者往后开
  // if (innerProd > 0) {}

  double epsilon = 1e-3;

  size_t numPoints = fp.x.size();
  int increase_count = 0;
  double dis_min = std::numeric_limits<double>::max();
  //改进方案
  for (size_t i = pre_index; i < numPoints; ++i) {
    if (std::abs(innerProd) < epsilon) {
      index = pre_index;
      break;
    }

    double temp_dis = std::pow(fp.x[i] - current_post_x, 2) +
                      std::pow(fp.y[i] - current_post_y, 2);
    if (temp_dis < dis_min) {
      dis_min = temp_dis;
      index = i;
      //如果后面出现更小的距离，则计数归零
      increase_count = 0;
    } else {
      ++increase_count;
    }

    //当最小的距离在后面10次求解过程中任然是最小值，那就认为这就是最佳匹配点
    if (increase_count > 10) {
      break;
    }
  }

  // 原始方案
  // for (size_t i = 0; i < numPoints; ++i) {
  //   double temp_dis = std::pow(fp.x[i] - current_post_x, 2) +
  //                     std::pow(fp.y[i] - current_post_y, 2);

  //   if (temp_dis < dis_min) {
  //     dis_min = temp_dis;
  //     index = i;
  //   }
  // }
}
}  // namespace cpprobotics