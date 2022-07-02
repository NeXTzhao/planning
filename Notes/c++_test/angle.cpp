//
// Created by next on 22-5-23.
//
#include <iostream>
#include <cmath>

using std::acos; // 反余弦函数
using std::sqrt; // 平方根函数
// 点结构体的定义
struct TPoint {
//    TPoint() {}

    TPoint(double x1, double y1) : x(x1), y(y1) {

    }

    double x;
    double y;
};

double getAngle(TPoint pSrc, TPoint p1, TPoint p2) {
    double angle = 0.0; // 夹角

    // 向量Vector a的(x, y)坐标
    double va_x = p1.x - pSrc.x;
    double va_y = p1.y - pSrc.y;

    // 向量b的(x, y)坐标
    double vb_x = p2.x - pSrc.x;
    double vb_y = p2.y - pSrc.y;

    double productValue = (va_x * vb_x) + (va_y * vb_y);  // 向量的乘积
    double va_val = sqrt(va_x * va_x + va_y * va_y);  // 向量a的模
    double vb_val = sqrt(vb_x * vb_x + vb_y * vb_y);  // 向量b的模
    double cosValue = productValue / (va_val * vb_val);      // 余弦公式

    // acos的输入参数范围必须在[-1, 1]之间，否则会"domain error"
    // 对输入参数作校验和处理
    if (cosValue < -1 && cosValue > -2)
        cosValue = -1;
    else if (cosValue > 1 && cosValue < 2)
        cosValue = 1;

    // acos返回的是弧度值，转换为角度值
    angle = acos(cosValue) * 180 / M_PI;
//    angle = acos(cosValue);

    return angle;
}

double getAngleNew(std::pair<double, double> p0, std::pair<double, double> p1, std::pair<double, double> p2) {
    double angle = 0.0; // 夹角

    // 向量Vector a的(x, y)坐标
    double va_x = p1.first - p0.first;
    double va_y = p1.second - p0.second;

    // 向量b的(x, y)坐标
    double vb_x = p2.first - p0.first;
//    double vb_x = p2.x - pSrc.x;
//    double vb_y = p2.y - pSrc.y;
    double vb_y = p2.second - p0.second;

    double productValue = (va_x * vb_x) + (va_y * vb_y);  // 向量的乘积
    double va_val = sqrt(va_x * va_x + va_y * va_y);  // 向量a的模
    double vb_val = sqrt(vb_x * vb_x + vb_y * vb_y);  // 向量b的模
    double cosValue = productValue / (va_val * vb_val);      // 余弦公式

    // acos的输入参数范围必须在[-1, 1]之间，否则会"domain error"
    // 对输入参数作校验和处理
    if (cosValue < -1 && cosValue > -2)
        cosValue = -1;
    else if (cosValue > 1 && cosValue < 2)
        cosValue = 1;

    // acos返回的是弧度值，转换为角度值
//    angle = acos(cosValue) * 180 / M_PI;
    angle = acos(cosValue);
    std::cout << "angle2:" << angle << std::endl;

    return angle;
}


int main() {

    TPoint p0{0.0, 0.0};
    TPoint p1{-1.0, -1.0};
    TPoint p2{1.0, 0.0};

    auto angle = getAngle(p0, p1, p2);
    std::cout << "angle:" << angle << std::endl;
    double angleNew = getAngleNew(std::make_pair(0.0, 0.0), std::make_pair(0.0, -1.0), std::make_pair(1.0, 0.0));
    std::cout << "angle1:" << angleNew << std::endl;

}