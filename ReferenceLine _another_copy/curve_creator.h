#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <utility>
#include <tuple>

#include "curve.h"

namespace reference_line {

    class CurveCreator {
        using Node = std::array<double, 4>;
        using Node1 = std::array<double, 5>;
        std::vector<Node> nodes_;

        std::vector<double> deltas_;
        std::vector<Node> solve_nodes;

        std::shared_ptr<Curve> curve_ = nullptr;

    public:
        CurveCreator(std::vector<double> a_xs, std::vector<double> a_ys);

        void solve();

        const std::shared_ptr<Curve> &curve() const { return curve_; }
    };

}  // namespace reference_line