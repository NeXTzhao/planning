#pragma once

#include <array>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace cpprobotics {

using Vec_f = std::vector<double>;
using Poi_f = std::array<double, 2>;
using Vec_Poi = std::vector<Poi_f>;

}; // namespace cpprobotics
