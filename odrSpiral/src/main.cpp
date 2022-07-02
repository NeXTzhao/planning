#include <cstdio>
#include <memory>
#include <vector>

#include "curve.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
    //    double s;
    double theta;
    double kappa;
    double x;
    double y;
    std::vector<double> SpiralX, SpiralY, SpiralTheta, SpiralKappa;

    double x0 = 1.0, y0 = 1.0, theta0 = 0.7853981633974483;
    double curvStart = 0.1, curvEnd = 0.25;
    auto curve = std::make_shared<spiral>(x0, y0, curvStart, curvEnd, theta0);

    for (double i = 0; i < 1; i += 0.1) {
        SpiralX.emplace_back(i);
        SpiralY.emplace_back(i);
    }
    double curveLength = 8;
    for (double s = 0; s < curveLength; s += 0.1) {
        curve->clothoid(curveLength, s, x, y, theta, kappa);
        SpiralX.emplace_back(x);
        SpiralY.emplace_back(y);
        SpiralTheta.emplace_back(theta);
    }


    /*****************************************************************************/
    plt::named_plot("Euler_Spiral_XY", SpiralX, SpiralY);
    plt::legend();
    plt::axis("equal");
    plt::figure();
    plt::named_plot("Euler_Spiral_Theta", SpiralTheta);
    plt::legend();
    //    plt::axis("equal");
    plt::show();
}
