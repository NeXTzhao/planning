#include <iostream>
#include <vector>
#include <memory>

#include "hc_cc_circle.hpp"
#include "utilities.hpp"
#include "cc00_reeds_shepp_state_space.hpp"

#define KAPPA 1.0                        // [1/m]
#define SIGMA 1.0                        // [1/m^2]
#define DISCRETIZATION 0.1               // [m]

using namespace steering;

double get_distance(State const& state1, State const& state2) {
  return sqrt(pow(state2.x - state1.x, 2) + pow(state2.y - state1.y, 2));
}

double get_path_length(std::vector<State> const& path) {
  double path_length = 0;
  State state1 = path.front();
  for (auto const& state2 : path) {
    path_length += get_distance(state1, state2);
    state1 = state2;
  }
  return path_length;
}

double get_path_length(std::vector<Control> const& controls) {
  double path_length = 0;
  for (auto const& control : controls) path_length += fabs(control.delta_s);
  return path_length;
}

int main(){
  // 连续曲率RS曲线

  CC00_Reeds_Shepp_State_Space cc00_rs_ss(KAPPA, SIGMA, DISCRETIZATION);
//  auto cc_rs = std::make_unique<CC00_Reeds_Shepp_State_Space>(KAPPA, SIGMA, DISCRETIZATION);
//  std::vector<State> cc00_rs_path = cc00_rs_ss.get_path(start, goal);
//  EXPECT_LT(fabs(cc00_rs_ss.get_distance(start, goal) -
//                 get_path_length(cc00_rs_path)),
//            EPS_DISTANCE);

  return 0;
};