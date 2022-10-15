/**
 * @file frenetlqr.cpp
 * @author Wang Dezhao (1282507109@qq.com)
 * @brief
 * @version 0.1
 * @date 2021-10-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cpprobotics_types_double.h"
#include "frenet_path_double.h"
#include "quintic_polynomial_double.h"

#define DT 0.1  // time tick [s]

using namespace cpprobotics;

ros::Publisher frenet_lqr_;
ros::Publisher path_pub_;
ros::Publisher trajectory_path;

nav_msgs::Path path;
nav_msgs::Path trajectorypath;

/**************************************************************************/

// t-t0经历的时间
double T = 50;

double xend = 80.0;
double yend = 30.0;

// 起始状态
std::array<double, 3> x_start{0.0, 0.0, 0.0};
std::array<double, 3> x_end{xend, 0.0, 0.0};
// 终点状态
std::array<double, 3> y_start{0.0, 0.0, 0.0};
std::array<double, 3> y_end{yend, 0.0, 0.0};

/**************************************************************************/

/**
 * 整车参数及状态
 */

// 纵向速度
double vx = 0.01;
// 横向速度
double vy = 0;  //质心侧偏角视为不变
// 轮胎侧偏刚度
double cf = -65494.663, cr = -115494.663;

// 前后悬架载荷
double mass_fl = 500, mass_fr = 500, mass_rl = 520, mass_rr = 520;
double mass_front = mass_fl + mass_fr;
double mass_rear = mass_rl + mass_rr;
double m = mass_front + mass_rear;

// 最大纵向加速度
double max_lateral_acceleration = 5.0;
// 最大制动减速度
double standstill_acceleration = -3.0;
// 轴距
double wheel_base = 3.8;
// 前轴中心到质心的距离
double a = wheel_base * (1.0 - mass_front / m);
// 后轴中心到质心的距离
double b = wheel_base * (1.0 - mass_rear / m);

// 车辆绕z轴转动的转动惯量
double Iz = std::pow(a, 2) * mass_front + std::pow(b, 2) * mass_rear;

// 轮胎最大转角(rad)
double wheel_max_degree = 0.6;

/**************************************************************************/

/**
 * @brief 计算四元数转换到欧拉角
 *
 * @param x
 * @param y
 * @param z
 * @param w
 * @return std::array<double, 3>
 */
std::array<double, 3> calQuaternionToEuler(const double x, const double y,
                                           const double z, const double w) {
  std::array<double, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  return calRPY;
}
/**************************************************************************/

/**
 * @brief 规划路径
 *
 */
FrenetPath fp;
void calc_frenet_paths() {
  // 纵向
  QuinticPolynomial lon_qp(x_start[0], x_start[1], x_start[2], x_end[0],
                           x_end[1], x_end[2], T);
  // 横向
  QuinticPolynomial lat_qp(y_start[0], y_start[1], y_start[2], y_end[0],
                           y_end[1], y_end[2], T, xend);

  for (double t = 0; t < T; t += DT) {
    double x = lon_qp.calc_point_x(t);
    double xd = lon_qp.calc_point_xd(t);
    double xdd = lon_qp.calc_point_xdd(t);

    fp.t.push_back(t);
    fp.x.push_back(x);
    fp.x_d.push_back(xd);
    fp.x_dd.push_back(xdd);

    double y_x_t = lat_qp.calc_point_y_x(x);
    double y_x_d = lat_qp.calc_point_y_x_d(x);
    double y_x_t_d = lat_qp.calc_point_y_t_d(y_x_d, xd);

    double y_x_dd = lat_qp.calc_point_y_x_dd(x);
    double y_x_t_dd = lat_qp.calc_point_y_t_dd(y_x_dd, xd, y_x_d, xdd);

    fp.y.push_back(y_x_t);
    fp.y_d.push_back(y_x_t_d);
    fp.y_dd.push_back(y_x_t_dd);
    // 压入航向角
    // fp.threat.push_back(lat_qp.calc_point_thetar(y_x_t_d, xd));

    // 压入曲率
    fp.k.push_back(lat_qp.calc_point_k(y_x_dd, y_x_d));
    // fp.k.push_back(lat_qp.calc_point_k(y_x_t_dd, y_x_t_d, xdd, xd));
  }
  int num = fp.x.size();
  for (int i = 0; i < num; i++) {
    double dy = fp.y[i + 1] - fp.y[i];
    double dx = fp.x[i + 1] - fp.x[i];
    fp.threat.push_back(lat_qp.calc_point_thetar(dy, dx));
  }
  // 最后一个道路航向角和前一个相同
  // fp.threat.push_back(fp.threat.back());
}
/**************************************************************************/

/**
 * @brief 寻找匹配点即距离最短的点
 *
 * @param current_post_x
 * @param current_post_y
 * @return int
 */
int index_ = 0;
int findTrajref(double current_post_x, double current_post_y) {
  int numPoints = fp.x.size();
  // double dis_min = std::pow(fp.x[0] - current_post_x, 2) +
  //                  std::pow(fp.y[0] - current_post_y, 2);
  double dis_min = std::numeric_limits<double>::max();

  int index = 0;
  for (int i = index; i < numPoints; i++) {
    double temp_dis = std::pow(fp.x[i] - current_post_x, 2) +
                      std::pow(fp.y[i] - current_post_y, 2);
    // printf("dis_min,temp_dis=%f,%f \n", dis_min, temp_dis);
    if (temp_dis < dis_min) {
      dis_min = temp_dis;
      index = i;
    }
  }
  index_ = index;
  // printf("index,numPoints=%d,%d \n", index, numPoints);
  return index;
}

/**
 * @brief 计算误差err和投影点的曲率
 *  1.先遍历找到匹配点
 *  2.通过匹配点近似求解投影点
 *    2.1 由投影点得到对应的航向角和曲率
 *
 * @param current_post_x
 * @param current_post_y
 * @param calRPY
 * @return std::array<double, 5>
 */
std::array<double, 5> cal_err_k(double current_post_x, double current_post_y,
                                std::array<double, 3> calRPY) {
  std::array<double, 5> err_k;
  int index = findTrajref(current_post_x, current_post_y);
  // 找到index后，开始求解投影点
  // Eigen::Vector2f tor;
  Eigen::Matrix<double, 2, 1> tor;
  tor << cos(fp.threat[index]), sin(fp.threat[index]);
  // Eigen::Vector2f nor;
  Eigen::Matrix<double, 2, 1> nor;
  nor << -sin(fp.threat[index]), cos(fp.threat[index]);

  // Eigen::Vector2f d_err;
  Eigen::Matrix<double, 2, 1> d_err;
  d_err << current_post_x - fp.x[index], current_post_y - fp.y[index];

  double phi = calRPY[2];

  // nor.transpose()对nor转置
  double ed = nor.transpose() * d_err;
  // double ed = -vx*sin();

  double es = tor.transpose() * d_err;

  // 投影点的threat角度
  double projection_point_threat = fp.threat[index] + fp.k[index] * es;

  // double phi = fp.threat[index];
  double ed_d = vy * cos(phi - projection_point_threat) +
                vx * sin(phi - projection_point_threat);
  // 计算ephi
  // double ephi = sin(phi - projection_point_threat);
  double ephi = phi - projection_point_threat;

  // 计算s_d
  double s_d = (vx * cos(phi - projection_point_threat) -
                vy * sin(phi - projection_point_threat)) /
               (1 - fp.k[index] * ed);
  double phi_d = vx * fp.k[index];
  double ephi_d = phi_d - fp.k[index] * s_d;

  // 计算投影点曲率k
  double projection_point_curvature = fp.k[index];

  err_k[0] = ed;
  err_k[1] = ed_d;
  err_k[2] = ephi;
  err_k[3] = ephi_d;
  err_k[4] = projection_point_curvature;

  return err_k;
}

/**
 * @brief 求解k系数
 *   1.首先用迭代法解黎卡提方程得到参数得到p矩阵
 *   2.将p带入k得到k值
 *   2.将得到的k带入u(n)=-kx(n)得到u也就是转角的控制量
 * @param A
 * @param B
 * @param Q
 * @param R
 * @return Eigen::RowVector4cf
 */
Eigen::Matrix<double, 1, 4> cal_dlqr(Eigen::Matrix4d A,
                                     Eigen::Matrix<double, 4, 1> B,
                                     Eigen::Matrix4d Q,
                                     Eigen::Matrix<double, 1, 1> R) {
  // 设置最大循环迭代次数
  int numLoop = 200;
  // 设置目标极小值
  double minValue = 10e-10;
  Eigen::Matrix4d p_old;
  p_old = Q;

  /*************************************/

  /**
   * 离散化状态方程
   *
   */
  double ts = 0.001;
  Eigen::Matrix4d eye;
  eye.setIdentity(4, 4);

  Eigen::Matrix4d Ad;
  Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
  Eigen::Matrix<double, 4, 1> Bd;
  Bd = B * ts;

  /*************************************/
  for (int i = 0; i < numLoop; i++) {
    // B.inverse()求逆
    Eigen::Matrix4d p_new = Ad.transpose() * p_old * Ad -
                            Ad.transpose() * p_old * Bd *
                                (R + Bd.transpose() * p_old * Bd).inverse() *
                                Bd.transpose() * p_old * Ad +
                            Q;
    // p.determinant()求行列式
    // if (std::abs((p_old - p_new).determinant()) <= minValue) {
    // cwiseAbs()求绝对值、maxCoeff()求最大系数
    if (fabs((p_new - p_old).maxCoeff()) < minValue) {
      p_old = p_new;
      break;
    }
    p_old = p_new;
  }
  Eigen::Matrix<double, 1, 4> k;
  // Eigen::RowVector4f;
  // 当两个超出范围的浮点数（即INF）进行运算时，运算结果会成为NaN。
  k = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
  // std::cout << "k:\n" << k << std::endl;
  // std::cout << "A: \n"
  //           << Ad << "\n B: \n"
  //           << Bd << "\n Q: \n"
  //           << Q << "\n R: \n"
  //           << R << "\n p: \n"
  //           << p_old << "\n k: \n"
  //           << k << std::endl;
  return k;
}

/**
 * @brief 计算k值
 *
 * @param err_k
 * @return Eigen::Matrix<double, 1, 4>
 */
Eigen::Matrix<double, 1, 4> cal_k(std::array<double, 5> err_k) {
  Eigen::Matrix4d A;
  A << 0, 1, 0, 0, 0, (cf + cr) / (m * vx), -(cf + cr) / m,
      (a * cf - b * cr) / (m * vx), 0, 0, 0, 1, 0,
      (a * cf - b * cr) / (Iz * vx), -(a * cf - b * cr) / Iz,
      (a * a * cf + b * b * cr) / (Iz * vx);

  // Eigen::Vector4f B;
  Eigen::Matrix<double, 4, 1> B;
  B << 0, -cf / m, 0, -a * cf / Iz;

  // Eigen::Matrix4f Q;
  // 设置成单位矩阵
  Eigen::Matrix4d Q;
  // Q.setIdentity(4, 4);
  Q(0, 0) = 60;
  Q(1, 1) = 1;
  Q(2, 2) = 1;
  Q(3, 3) = 1;

  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 35.0;
  // MatrixXd矩阵只能用(),VectorXd不仅能用()还能用[]
  Eigen::Matrix<double, 1, 4> k = cal_dlqr(A, B, Q, R);

  return k;
}

/**
 * @brief 计算前馈环节
 *
 * @param k
 * @param err_k
 * @return double
 */
double cal_forword_angle(Eigen::Matrix<double, 1, 4> k,
                         std::array<double, 5> err_k) {
  double k3 = k[2];
  // 不足转向系数
  double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);

  //投影点的曲率final_path.k[index]
  double point_curvature = err_k[4];
  double forword_angle =
      wheel_base * point_curvature + kv * vx * vx * point_curvature -
      k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);
  // double forword_angle =
  //     projection_point_curvature *
  //     (a + b - b * k3 -
  //      (m * std::pow(vx, 2) / (a + b)) * (b / cf + a * k3 / cr - a / cr));
  return forword_angle;
}

/**
 * @brief 计算前轮转角u
 *
 * @param k
 * @param forword_angle
 * @param err_k
 */
double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle,
                 std::array<double, 5> err_k) {
  Eigen::Matrix<double, 4, 1> err;
  err << err_k[0], err_k[1], err_k[2], err_k[3];
  double angle = -k * err + forword_angle;

  return angle;
}

/**
 * @brief 限制前轮最大转角
 *
 * @param angle
 * @return double
 */
double limitSterringAngle(double value, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

/**
 * @brief 统一调用各个子函数
 *
 * @param currentPositionX
 * @param currentPositionY
 * @param cal_RPY
 * @return double
 */
double theta_angle(double currentPositionX, double currentPositionY,
                   std::array<double, 3> cal_RPY) {
  std::array<double, 5> err_k =
      cal_err_k(currentPositionX, currentPositionY, cal_RPY);
  Eigen::Matrix<double, 1, 4> k = cal_k(err_k);

  double forword_angle = cal_forword_angle(k, err_k);
  double tempangle = cal_angle(k, forword_angle, err_k);
  double angle =
      limitSterringAngle(tempangle, -wheel_max_degree, wheel_max_degree);
  printf("angle,forword_angle=%.3f,%.3f\n", angle, forword_angle);

  return angle;
}

void velocityCall(const geometry_msgs::TwistStamped &carWaypoint) {
  //错误写法 carVelocity = carWaypoint.linear.x;
  vx = carWaypoint.twist.linear.x;
}

void poseCallback(const geometry_msgs::PoseStamped &currentWaypoint) {
  double currentPositionX = currentWaypoint.pose.position.x;
  double currentPositionY = currentWaypoint.pose.position.y;
  double currentPositionZ = 0.0;

  double currentQuaternionX = currentWaypoint.pose.orientation.x;
  double currentQuaternionY = currentWaypoint.pose.orientation.y;
  double currentQuaternionZ = currentWaypoint.pose.orientation.z;
  double currentQuaternionW = currentWaypoint.pose.orientation.w;

  std::array<double, 3> cal_RPY =
      calQuaternionToEuler(currentQuaternionX, currentQuaternionY,
                           currentQuaternionZ, currentQuaternionW);

  double theta = theta_angle(currentPositionX, currentPositionY, cal_RPY);

  int numpoints = fp.x.size();
  if (index_ < numpoints - 2) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 8;
    vel_msg.angular.z = theta;
    frenet_lqr_.publish(vel_msg);

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "world";
    trajectorypath.poses.push_back(this_pose_stamped);

    trajectory_path.publish(trajectorypath);
  } else {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    frenet_lqr_.publish(vel_msg);
  }
}

int main(int argc, char **argv) {
  //创建节点
  ros::init(argc, argv, "lqr");
  //创建节点句柄
  ros::NodeHandle a;
  //创建Publisher，发送经过lqr计算后的转角及速度
  frenet_lqr_ = a.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 20);

  //初始化五次多项式轨迹
  calc_frenet_paths();

  int Num = fp.x.size();
  for (int i = 0; i < Num; i++) {
    printf("x,y,th,k,i=%.3f,%.3f,%.3f,%f,%d \n", fp.x[i], fp.y[i], fp.threat[i],
           fp.k[i], i);
  }

  /**************************************************************/
  // 发布规划轨迹
  path_pub_ = a.advertise<nav_msgs::Path>("rviz_path", 20, true);
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  int sNum = fp.x.size();
  for (int i = 0; i < sNum; i++) {
    pose.pose.position.x = fp.x[i];
    pose.pose.position.y = fp.y[i];
    pose.pose.position.z = 0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;
    path.poses.push_back(pose);
  }
  path_pub_.publish(path);

  /**************************************************************/
  //发布小车运动轨迹
  trajectory_path = a.advertise<nav_msgs::Path>("trajector_ypath", 20, true);
  trajectorypath.header.frame_id = "world";
  trajectorypath.header.stamp = ros::Time::now();

  /**************************************************************/

  ros::Subscriber carVel = a.subscribe("/smart/velocity", 20, velocityCall);
  ros::Subscriber carPose = a.subscribe("/smart/rear_pose", 20, poseCallback);
  ros::spin();

  return 0;
};