/*********************************************************************
 *  Copyright (c) 2017 Robert Bosch GmbH.
 *  All rights reserved.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 * *********************************************************************/

#ifndef EKF_HPP
#define EKF_HPP

#include "steering_functions.hpp"
#include <Eigen/Dense>

namespace steering {

typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 2> Matrix32d;
typedef Eigen::Matrix<double, 2, 3> Matrix23d;

class EKF {
 public:
  /** Constructor */
  EKF();

  /** \brief Sets the parameters required by the EKF */
  void set_parameters(Motion_Noise const& motion_noise,
                      Measurement_Noise const& measurement_noise,
                      Controller const& _controller);

  /** \brief Converts a covariance given by a double array to an Eigen matrix */
  Matrix3d covariance_to_eigen(double const covariance[16]) const;

  /** \brief Converts a covariance given by an Eigen matrix to a double array */
  void eigen_to_covariance(Matrix3d const& covariance_eigen,
                           double covariance[16]) const;

  /** \brief Computes the Jacobians of the motion equations with respect to the
   * state and control */
  void get_motion_jacobi(State const& state, Control const& control,
                         double integration_step, Matrix3d& F_x,
                         Matrix32d& F_u) const;

  /** \brief Computes the Jacobian of the observation equations with respect to
   * the state */
  Matrix3d get_observation_jacobi() const;

  /** \brief Returns the motion covariance in control space */
  Matrix2d get_motion_covariance(State const& state, Control const& control,
                                 double integration_step) const;

  /** \brief Returns the observation covariance */
  Matrix3d get_observation_covariance() const;

  /** \brief Returns the gain of the controller */
  Matrix23d get_controller_gain(Control const& control) const;

  /** \brief Returns the rotation matrix from a global frame to a local frame */
  Matrix3d get_rotation_matrix(double angle) const;

  /** \brief Predicts the covariances based on the paper:
      Rapidly-exploring random belief trees for motion planning under
     uncertainty, A. Bry and N. Roy, IEEE ICRA 2011 */
  void predict(State_With_Covariance const& state, Control const& control,
               double integration_step,
               State_With_Covariance& state_pred) const;

  /** \brief Predicts the covariances */
  void update(State_With_Covariance const& state_pred,
              State_With_Covariance& state_corr) const;

  /** \brief Overload operator new for fixed-size vectorizable Eigen member
   * variable */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  /** \brief Motion noise */
  Motion_Noise motion_noise_;

  /** \brief Measurement noise */
  Measurement_Noise measurement_noise_;

  /** \brief Feedback controller */
  Controller controller_;

  /** \brief Identity matrix */
  Matrix3d I_;
};

}  // namespace steering

#endif
