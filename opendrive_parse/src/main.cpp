#include <stdio.h>

#include <vector>
#include "matplotlibcpp.h"
#include "../include/Geometries/Line.h"
#include "../include/Geometries/Spiral.h"
#include "../include/Geometries/Arc.h"

namespace plt = matplotlibcpp;
using namespace odr;
// <geometry s="0.0000000000000000e+00" x="-3.0000000000000000e+01" y="4.2500000000000000e+00" hdg="1.3125944268194595e-16" length="6.7665868569339089e+00">
//     <line/>
// </geometry>
// <geometry s="6.7665868569339089e+00" x="-2.3233413143066091e+01" y="4.2500000000000009e+00" hdg="2.2204460492503131e-16" length="1.3473997480893811e+01">
//     <spiral curvStart="-0.0000000000000000e+00" curvEnd="-6.3846739791627374e-03"/>
// </geometry>
// <geometry s="2.0240584337827720e+01" x="-9.7619083596721836e+00" y="4.0568374168342416e+00" hdg="-4.3013540554541763e-02" length="1.3473997480893807e+01">
//     <arc curvature="-6.3846739791627374e-03"/>

// </geometry>
// <geometry s="3.3714581818721527e+01" x="3.6581225294544950e+00" y="2.8994948890710077e+00" hdg="-1.2904062166610863e-01" length="1.3473997480893811e+01">
//         <spiral curvStart="-6.3846739791627374e-03" curvEnd="-0.0000000000000000e+00"/>
// </geometry>
// <geometry s="4.7188579299615341e+01" x="1.6963793395133585e+01" y="7.8339571210420766e-01" hdg="-1.7205416222810044e-01" length="1.3110166801430049e+01">
//     <line/>
// </geometry>

int main() {
    double line_s = 0.0000000000000000e+00, line_x = -3.0000000000000000e+01, line_y = 4.2500000000000000e+00, line_hdg = 1.3125944268194595e-16, line_length = 6.7665868569339089e+00;
    double spiral_s = 6.7665868569339089e+00, spiral_x = -2.3233413143066091e+01, spiral_y = 4.2500000000000009e+00, spiral_hdg = 2.2204460492503131e-16, spiral_length = 1.3473997480893811e+01,
            spiral_curvStart = -0.0000000000000000e+00, spiral_curvEnd = -6.3846739791627374e-03;
    double arc_s = 2.0240584337827720e+01, arc_x = -9.7619083596721836e+00, arc_y = 4.0568374168342416e+00, arc_hdg = -4.3013540554541763e-02, arc_length = 1.3473997480893807e+01,
            arc_curvature = -6.3846739791627374e-03;

    auto line = std::make_shared<Line>(line_s, line_x, line_y, line_hdg, line_length);
    auto spiral = std::make_shared<Spiral>(spiral_s, spiral_x, spiral_y, spiral_hdg, spiral_length, spiral_curvStart,
                                           spiral_curvEnd);
    auto arc = std::make_shared<Arc>(arc_s, arc_x, arc_y, arc_hdg, arc_length, arc_curvature);

    std::vector<double> ordX, ordY;

    for (double s = 0; s < line_length; s += 0.1) {
        auto xy = line->get_xy(s);
        ordX.emplace_back(xy.at(0));
        ordY.emplace_back(xy.at(1));
//        printf("line_x=%f,line_y=%f\n", xy.at(0), xy.at(1));
    }
    double spiral_x0 = ordX.back();

    std::cout << '\n';
    for (double s = spiral_s; s < spiral_s+spiral_length; s += 0.1) {
        auto xy = spiral->get_xy(s);
        double x = xy.at(0);
        if (x > spiral_x0) {
            ordX.emplace_back(xy.at(0));
            ordY.emplace_back(xy.at(1));
        }
//        printf("spiral_x=%f,spiral_y=%f\n", xy.at(0), xy.at(1));
    }
    std::cout << '\n';
    for (double s = arc_s; s < arc_s+arc_length; s += 0.1) {
        auto xy = arc->get_xy(s);
        ordX.emplace_back(xy.at(0));
        ordY.emplace_back(xy.at(1));
//        printf("arc_x=%f,arc_y=%f\n", xy.at(0), xy.at(1));
    }

    /*****************************************************************************/
    plt::named_plot("ord_XY", ordX, ordY);
    plt::legend();
    plt::axis("equal");
    plt::show();
}