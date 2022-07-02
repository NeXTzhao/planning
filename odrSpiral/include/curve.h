#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

/**
 *  起始坐标为0，计算在标准坐标下的x,y
 * @param s      螺旋线长度
 * @param curvDot   曲率的一阶导 [1/m2]  ==> dk = a * ds; 常数 a 称为回旋曲线的平坦度
 * @param x      局部螺旋线坐标系下的x位置 [m]
 * @param y      局部螺旋线坐标系下的y位置 [m]
 * @param theta      s处的切线方向(即 theta) [rad]
 */

//extern void eulerSpiral(double s, double curvDot, double *x, double *y, double *theta);
//extern void transferCoordinate(Eigen::Matrix<double, 2, 2> &trans, double theta);
//void eulerSpiral(double s, double curvDot, double *x, double *y, double *theta, double *start_theta, double *kappa);
//
//extern void clothoid(double curveStart, double curveEnd, double curveLength, double distance, double &x, double &y, double &theta, double &start_theta, double &kappa);

class spiral {
public:
    spiral(double x, double y, double curvStart, double curvEnd, double theta) : start_x_(x), start_y_(y), curveStart_(curvStart), curveEnd_(curvEnd), start_theta_(theta) {}

//    void clothoid(double curveStart, double curveEnd, double curveLength, double distance, double &x, double &y, double &theta, double &kappa);
    void clothoid(double curveLength, double distance, double &x, double &y, double &theta, double &kappa);
private:
    /**
     * 坐标转换矩阵
     * @param trans
     * @param theta
     */
    static void transferCoordinate(Eigen::Matrix<double, 2, 2> &trans, double theta) {
        trans << cos(theta), -sin(theta), sin(theta), cos(theta);
    }

    /**
     * 计算x,y theta,kappa
     * @param s
     * @param curvDot
     * @param x
     * @param y
     * @param theta
     * @param start_theta
     * @param kappa
     */
    void eulerSpiral_(double s, double curvDot, double *x, double *y, double *theta, double *kappa);

private:
    double start_x_, start_y_;
    double curveStart_, curveEnd_;
    double start_theta_;
};
