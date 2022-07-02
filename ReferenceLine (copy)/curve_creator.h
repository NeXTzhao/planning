#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <tuple>
#include <utility>

#include "curve.h"

namespace reference_line {
    class CurveCreator {
        using Node = std::array<double, 4>;
        using Node1 = std::array<double, 5>;

        std::vector<Node> nodes_;
        std::vector<Node1> solve_nodes;

        std::vector<double> deltas_;

        std::shared_ptr<Curve> curve_ = nullptr;

    public:
        CurveCreator(const std::vector<double> &a_xs, const std::vector<double> &a_ys);

        void solve();

        const std::shared_ptr<Curve> &curve() const { return curve_; }
    };

}// namespace reference_line