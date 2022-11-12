//
// Created by next on 22-7-15.
//

#pragma once
#include <vector>

struct ReedSheppPath {
    std::vector<double> segs_lengths;
    std::vector<char> segs_types;
    double total_length = 0.0;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;
    // true for driving forward and false for driving backward
    std::vector<bool> gear;
};

class TurnParams {

};


