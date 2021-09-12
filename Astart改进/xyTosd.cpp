#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>

using namespace std;

/**
 * 根据笛卡尔坐标的 x,y 求到 Frenet下的 s 和距离ｄ
*/
vector<double> cartesian_to_frenet1D(double rs, double rx, double ry, double rtheta, double x, double y)
{
    vector<double> s_array = {0};

    // 微分
    double dx = x - rs;
    double dy = y - rx;

    // 计算方向 先将角度化为弧度
    rtheta = rtheta * M_PI / 180;
    int corss_r_x = (cos(rtheta) * dy - sin(rtheta) * dx) > 0 ? 1 : -1;

    // 计算距离 方向 * 两点间的距离
    double distance＝sign(corss_r_x) * sqrt(dx * dx + dy * dy);
    s_array[0] = distance;
    s_array[1] = rs;

    return s_array;
}
