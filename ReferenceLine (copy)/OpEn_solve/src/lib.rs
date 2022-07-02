// use std::f64::consts::FRAC_PI_2;
use optimization_engine::{constraints::*, panoc::*, core::*};

// extern "C" {
//   fn cos(input: f64) -> f64;
//   fn sin(input: f64) -> f64;
//   fn pow(input: f64, exp: f64) -> f64;
// }

// ---Private Constants----
/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 1e-03;

/// LBFGS memory
const LBFGS_MEMORY: usize = 350;

/// Maximum number of iterations of the solver
const MAX_ITERATIONS: usize = 500;


/// Solver interface
#[no_mangle]
#[warn(non_snake_case)]
pub unsafe extern "C" fn open_solve(thetas: *const f64, kappas: *const f64, xs: *const f64, ys: *const f64, deltas: *const f64, get_solve_value: *mut f64, size: usize) {
  let theta = std::slice::from_raw_parts(thetas, size);
  let kappa = std::slice::from_raw_parts(kappas, size);
  let x = std::slice::from_raw_parts(xs, size);
  let y = std::slice::from_raw_parts(ys, size);
  let delta = std::slice::from_raw_parts(deltas, size - 1);
  let get_value = std::slice::from_raw_parts_mut(get_solve_value, 5 * size);

  // get starting point
  let u_size = 5 * size - 1;
  // println! {"u_size={}", u_size};

  let mut u = vec![0.0; u_size];
  let variable_index = size * 4;
  for i in 0..size {
    let index = 4 * i;
    u[index] = theta[i];
    u[index + 1] = kappa[i];
    u[index + 2] = x[i];
    u[index + 3] = y[i];
    if i <= size - 2 {
      u[variable_index + i] = delta[i];
    }
  }

  // get cost function
  let xy_cost_function = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
    *c = err_x_y_cost(&u, size);
    Ok(())
  };

  // get bounds info
  let mut variate_lower_bound = vec![-f64::INFINITY; u_size];
  let mut variate_upper_bound = vec![f64::INFINITY; u_size];
  // let delta_index = size * 4;
  let fault_dis = 0.1;
  /* for theta kappa x y */
  for i in 0..size {
    let index = i * 4;
    // theta
    // variate_lower_bound[index] = -f64::INFINITY;
    // variate_upper_bound[index] = f64::INFINITY;
    // // kappa
    // variate_lower_bound[index + 1] = -f64::INFINITY;
    // variate_upper_bound[index + 1] = f64::INFINITY;
    // x
    variate_lower_bound[index + 2] = u[index + 2] - fault_dis;
    variate_upper_bound[index + 2] = u[index + 2] + fault_dis;
    // y
    variate_lower_bound[index + 3] = u[index + 3] - fault_dis;
    variate_upper_bound[index + 3] = u[index + 3] + fault_dis;
    // if i <= size - 2 {
    //   variate_lower_bound[delta_index + i] = -f64::INFINITY;
    //   variate_upper_bound[delta_index + i] = f64::INFINITY;
    // }
  }

// get Jacobian matrix
  let xy_cost_function_grad = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError>{
    cost_function_jacobians(u, size, grad);
    Ok(())
  };


  let mut panoc_cache = PANOCCache::new(u_size, EPSILON_TOLERANCE, LBFGS_MEMORY);
  let bounds = Rectangle::new(Some(&variate_lower_bound), Some(&variate_upper_bound));
  let problem = Problem::new(&bounds, xy_cost_function_grad, xy_cost_function);
  let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(MAX_ITERATIONS);
  let status = panoc.solve(&mut u).unwrap();
  println!("Panoc status: {:#?}", status);
  // println!("Panoc solution: {:#?}", u);

  for i in 0..size {
    if i == size - 1 {
      let index = 5 * size - 1;
      let index2 = 4 * size;
      get_value[index - 4] = u[index2 - 4];
      get_value[index - 3] = u[index2 - 3];
      get_value[index - 2] = u[index2 - 2];
      get_value[index - 1] = u[index2 - 1];
      get_value[index] = u[index - 1];
      // println! {"theta={},kappa={},x={},y={}", u[index2 - 4], u[index2 - 3], u[index2 - 2], u[index2 - 1]};
    } else {
      let index = i * 4;
      let index1 = 4 * size;
      let index2 = i * 5;
      get_value[index2] = u[index];
      get_value[index2 + 1] = u[index + 1];
      get_value[index2 + 2] = u[index + 2];
      get_value[index2 + 3] = u[index + 3];
      get_value[index2 + 4] = u[index1 + i];
      // println! {"theta={},kappa={},x={},y={},s={}", u[index], u[index + 1], u[index + 2], u[index + 3], u[index1 + i]};
    }
  }
}

unsafe fn err_x_y_cost(u: &[f64], size: usize) -> f64 {
  let index2 = 4 * size;
  let mut cost_value = 0.0;
  for i in 0..size - 1 {
    let index = i * 4;
    let index1 = (i + 1) * 4;

    let theta0 = u[index];
    let kappa0 = u[index + 1];
    let x0 = u[index + 2];
    let y0 = u[index + 3];

    let theta1 = u[index1];
    let kappa1 = u[index1 + 1];
    let x1 = u[index1 + 2];
    let y1 = u[index1 + 3];
    // println!("x0={}, x1={}, y0={}, y1={}", u[index + 2], u[index1 + 2], u[index + 3], u[index1 + 3]);

    let delta = u[index2 + i];

    let dx = 0.06474248308443546 * delta * (
      0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
          0.0019095507544770018 * theta1).cos() + 0.13985269574463816 * delta * (
      0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
          0.045787770837386436 * theta1).cos() + 0.1909150252525593 * delta * (
      0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
          0.2123278542966876 * theta1).cos() + 0.20897959183673434 * delta * (
      0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() +
        0.1909150252525593 * delta * (
          0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).cos() +
        0.13985269574463816 * delta * (
          0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos() +
        0.06474248308443546 * delta * (
          0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos();
    let dy = 0.06474248308443546 * delta * (
      0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
          0.0019095507544770018 * theta1).sin() + 0.13985269574463816 * delta * (
      0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
          0.045787770837386436 * theta1).sin() + 0.1909150252525593 * delta * (
      0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
          0.2123278542966876 * theta1).sin() + 0.20897959183673434 * delta * (
      0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
        0.1909150252525593 * delta * (
          0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).sin() +
        0.13985269574463816 * delta * (
          0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin() +
        0.06474248308443546 * delta * (
          0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin();

    cost_value += (x1 - x0 - dx).powi(2) + (y1 - y0 - dy).powi(2);
  }
  cost_value
}

unsafe fn cost_function_jacobians(u: &[f64], size: usize, jacobians: &mut [f64]) {
  let mut jacobians_index = 0;
  let index2 = 4 * size;

  for i in 0..size {
    if i == 0 {
      /* start points */
      let theta0 = u[0];
      let kappa0 = u[1];
      let x0 = u[2];
      let y0 = u[3];

      let theta1 = u[4];
      let kappa1 = u[5];
      let x1 = u[6];
      let y1 = u[7];

      let delta = u[4 * size];
      // theta0
      jacobians[jacobians_index] = 2.0 * (-0.06461885402701487 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
          0.0019095507544770018 * theta1).cos() - 0.13344915256089193 * delta *
          (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos()
          - 0.15037844758768545 * delta * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
          0.2123278542966876 * theta1).cos() - 0.10448979591836717 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.040536577664873834 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.0064035431837461835 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
          - 0.00012362905742060293 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
          (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
              (0.06461885402701487 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
                  0.0019095507544770018 * theta1).sin() + 0.13344915256089193 * delta *
                  (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin()
                  + 0.15037844758768545 * delta * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
                  0.2123278542966876 * theta1).sin() + 0.10448979591836717 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
                  0.040536577664873834 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                      0.7876721457033125 * theta1).sin() + 0.0064035431837461835 * delta *
                  (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
                  + 0.00012362905742060293 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
                  0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // kappa0
      jacobians[jacobians_index] = 2.0 * (-0.0015646651174168634 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
          0.013704131501108207 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
              0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos() -
          0.028023652733535475 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
              0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
          0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.011843686434051922 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
              0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).cos() -
          0.0020338944549318037 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
              0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos() -
          0.00004085411269717685 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
          (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
              (0.0015646651174168634 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
                  0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() +
                  0.013704131501108207 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
                      0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin() +
                  0.028023652733535475 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
                      0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() +
                  0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
                  0.011843686434051922 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
                      0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).sin() +
                  0.0020338944549318037 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
                      0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin() +
                  0.00004085411269717685 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
                      0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // x0
      jacobians[jacobians_index] = -2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos());
      jacobians_index = jacobians_index + 1;
      // y0
      jacobians[jacobians_index] = -2.0 * (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // println! {"start_jacobians_index={}", jacobians_index};
    } else if i == size - 2 {
      /* end points */

      let theta0 = u[index2 - 8];
      let kappa0 = u[index2 - 7];
      let x0 = u[index2 - 6];
      let y0 = u[index2 - 5];

      let theta1 = u[index2 - 4];
      let kappa1 = u[index2 - 3];
      let x1 = u[index2 - 2];
      let y1 = u[index2 - 1];

      let delta = u[5 * size - 2];
      // println! {"end0_jacobians_index={}", jacobians_index};
      // theta_i+1
      jacobians[jacobians_index] = 2.0 * (-0.00012362905742059827 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
          0.0019095507544770018 * theta1).cos() - 0.006403543183746221 * delta *
          (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos()
          - 0.04053657766487384 * delta * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
          0.2123278542966876 * theta1).cos() - 0.10448979591836717 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.15037844758768545 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13344915256089196 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
          - 0.06461885402701487 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
          (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
              (0.00012362905742059827 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
                  0.0019095507544770018 * theta1).sin() + 0.006403543183746221 * delta *
                  (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin()
                  + 0.04053657766487384 * delta * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
                  0.2123278542966876 * theta1).sin() + 0.10448979591836717 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
                  0.15037844758768545 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                      0.7876721457033125 * theta1).sin() + 0.13344915256089196 * delta *
                  (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
                  + 0.06461885402701487 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
                  0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // theta_i+1
      jacobians[jacobians_index] = 2.0 * (0.000040854112697175375 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() +
          0.0020338944549318097 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
              0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos() +
          0.01184368643405193 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
              0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() +
          0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() +
          0.028023652733535482 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
              0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).cos() +
          0.013704131501108207 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
              0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos() +
          0.0015646651174168603 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
          (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
              (-0.000040854112697175375 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
                  0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
                  0.0020338944549318097 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
                      0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin() -
                  0.01184368643405193 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
                      0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
                  0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
                  0.028023652733535482 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
                      0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).sin() -
                  0.013704131501108207 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
                      0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin() -
                  0.0015646651174168603 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
                      0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // x_i+1
      jacobians[jacobians_index] = 2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos());
      jacobians_index = jacobians_index + 1;
      // y_i+1
      jacobians[jacobians_index] = 2.0 * (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin());
      jacobians_index = jacobians_index + 1;
      // println! {"end1_jacobians_index={}", jacobians_index};
    } else {
      let index = (i - 1) * 4;
      let index1 = i * 4;
      let index3 = (i + 1) * 4;

      let theta0 = u[index];
      let kappa0 = u[index + 1];
      let x0 = u[index + 2];
      let y0 = u[index + 3];

      let theta1 = u[index1];
      let kappa1 = u[index1 + 1];
      let x1 = u[index1 + 2];
      let y1 = u[index1 + 3];

      let theta2 = u[index3];
      let kappa2 = u[index3 + 1];
      let x2 = u[index3 + 2];
      let y2 = u[index3 + 3];

      let delta = u[index2 + i - 1];
      let delta1;
      if i <= size - 2 {
        delta1 = u[index2 + i];
      } else {
        delta1 = u[index2 + i - 1];
      }
      // theta_i+1
      jacobians[jacobians_index] = 2.0 * (delta * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos()
          - 0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
              0.9542122291626138 * theta1).cos() - 0.06474248308443546 * delta *
          (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
              0.9980904492455229 * theta1).cos()) * (0.00012362905742059827 *
          (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
              0.0019095507544770018 * theta1).sin() + 0.006403543183746221 *
          (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).sin() + 0.04053657766487384 *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin()
          + 0.10448979591836717 * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
          0.15037844758768545 * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).sin() + 0.13344915256089196 *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
              0.9542122291626138 * theta1).sin() + 0.06461885402701487 *
          (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
              0.9980904492455229 * theta1).sin()) + delta * (-0.00012362905742059827 *
          (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
              0.0019095507544770018 * theta1).cos() - 0.006403543183746221 *
          (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).cos() - 0.04053657766487384 *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos()
          - 0.10448979591836717 * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.15037844758768545 * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13344915256089196 *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
              0.9542122291626138 * theta1).cos() - 0.06461885402701487 *
          (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
              0.9980904492455229 * theta1).cos()) * (-y0 + y1 - 0.06474248308443546 * delta *
          (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
              0.0019095507544770018 * theta1).sin() - 0.13985269574463816 * delta *
          (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin()
          - 0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
              0.9542122291626138 * theta1).sin() - 0.06474248308443546 * delta *
          (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
              0.9980904492455229 * theta1).sin()) + delta1 * (-x1 + x2 -
          0.06474248308443546 * delta1 * (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
              0.998090449245523 * theta1 + 0.0019095507544770018 * theta2).cos() -
          0.13985269574463816 * delta1 * (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 +
              0.9542122291626136 * theta1 + 0.045787770837386436 * theta2).cos() -
          0.1909150252525593 * delta1 * (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
              0.2123278542966876 * theta2).cos() - 0.20897959183673434 * delta1 *
          (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).cos() -
          0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
              0.7876721457033125 * theta2).cos() - 0.13985269574463816 * delta1 *
          (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
              0.9542122291626138 * theta2).cos() - 0.06474248308443546 * delta1 *
          (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
              0.9980904492455229 * theta2).cos()) * (0.06461885402701487 *
          (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
              0.0019095507544770018 * theta2).sin() + 0.13344915256089193 *
          (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
              0.045787770837386436 * theta2).sin() + 0.15037844758768545 *
          (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
              0.2123278542966876 * theta2).sin() + 0.10448979591836717 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).sin() +
          0.040536577664873834 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
              0.7876721457033125 * theta2).sin() + 0.0064035431837461835 *
          (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
              0.9542122291626138 * theta2).sin() + 0.00012362905742060293 *
          (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
              0.9980904492455229 * theta2).sin()) + delta1 * (-0.06461885402701487 *
          (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
              0.0019095507544770018 * theta2).cos() - 0.13344915256089193 *
          (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
              0.045787770837386436 * theta2).cos() - 0.15037844758768545 *
          (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
              0.2123278542966876 * theta2).cos() - 0.10448979591836717 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).cos() -
          0.040536577664873834 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
              0.7876721457033125 * theta2).cos() - 0.0064035431837461835 *
          (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
              0.9542122291626138 * theta2).cos() - 0.00012362905742060293 *
          (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
              0.9980904492455229 * theta2).cos()) * (-y1 + y2 - 0.06474248308443546 * delta1 *
          (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
              0.0019095507544770018 * theta2).sin() - 0.13985269574463816 * delta1 *
          (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
              0.045787770837386436 * theta2).sin() - 0.1909150252525593 * delta1 *
          (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
              0.2123278542966876 * theta2).sin() - 0.20897959183673434 * delta1 *
          (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).sin() -
          0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
              0.7876721457033125 * theta2).sin() - 0.13985269574463816 * delta1 *
          (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
              0.9542122291626138 * theta2).sin() - 0.06474248308443546 * delta1 *
          (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
              0.9980904492455229 * theta2).sin()));
      jacobians_index = jacobians_index + 1;
      // kappa_i+1
      jacobians[jacobians_index] = 2.0 * (0.000040854112697175375 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() +
          0.0020338944549318097 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
              0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos() +
          0.01184368643405193 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
              0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() +
          0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() +
          0.028023652733535482 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
              0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).cos() +
          0.013704131501108207 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
              0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos() +
          0.0015646651174168603 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
          (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
              0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
              0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                  0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
              (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
              0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
              0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                  0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
              (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
              - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
              0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
              (-0.000040854112697175375 * delta.powi(2) * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
                  0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
                  0.0020338944549318097 * delta.powi(2) * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
                      0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin() -
                  0.01184368643405193 * delta.powi(2) * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
                      0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
                  0.026122448979591793 * delta.powi(2) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
                  0.028023652733535482 * delta.powi(2) * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
                      0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).sin() -
                  0.013704131501108207 * delta.powi(2) * (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
                      0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin() -
                  0.0015646651174168603 * delta.powi(2) * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
                      0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
          2.0 * (-0.0015646651174168634 * delta1.powi(2) * (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
              0.998090449245523 * theta1 + 0.0019095507544770018 * theta2).cos() -
              0.013704131501108207 * delta1.powi(2) * (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 +
                  0.9542122291626136 * theta1 + 0.045787770837386436 * theta2).cos() -
              0.028023652733535475 * delta1.powi(2) * (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 +
                  0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).cos() -
              0.026122448979591793 * delta1.powi(2) * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).cos() -
              0.011843686434051922 * delta1.powi(2) * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 +
                  0.21232785429668755 * theta1 + 0.7876721457033125 * theta2).cos() -
              0.0020338944549318037 * delta1.powi(2) * (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 +
                  0.045787770837386166 * theta1 + 0.9542122291626138 * theta2).cos() -
              0.00004085411269717685 * delta1.powi(2) * (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 +
                  0.001909550754477074 * theta1 + 0.9980904492455229 * theta2).cos()) *
              (-y1 + y2 - 0.06474248308443546 * delta1 * (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
                  0.998090449245523 * theta1 + 0.0019095507544770018 * theta2).sin() -
                  0.13985269574463816 * delta1 * (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
                      0.045787770837386436 * theta2).sin() - 0.1909150252525593 * delta1 *
                  (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).sin()
                  - 0.20897959183673434 * delta1 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).sin() -
                  0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
                      0.7876721457033125 * theta2).sin() - 0.13985269574463816 * delta1 *
                  (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
                      0.9542122291626138 * theta2).sin() - 0.06474248308443546 * delta1 *
                  (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
                      0.9980904492455229 * theta2).sin()) + 2.0 * (-x1 + x2 - 0.06474248308443546 * delta1 *
          (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
              0.0019095507544770018 * theta2).cos() - 0.13985269574463816 * delta1 *
          (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
              0.045787770837386436 * theta2).cos() - 0.1909150252525593 * delta1 *
          (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).cos()
          - 0.20897959183673434 * delta1 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).cos() -
          0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
              0.7876721457033125 * theta2).cos() - 0.13985269574463816 * delta1 *
          (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
              0.9542122291626138 * theta2).cos() - 0.06474248308443546 * delta1 *
          (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
              0.9980904492455229 * theta2).cos()) * (0.0015646651174168634 * delta1.powi(2) *
          (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
              0.0019095507544770018 * theta2).sin() + 0.013704131501108207 * delta1.powi(2) *
          (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
              0.045787770837386436 * theta2).sin() + 0.028023652733535475 * delta1.powi(2) *
          (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).sin()
          + 0.026122448979591793 * delta1.powi(2) * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).sin() +
          0.011843686434051922 * delta1.powi(2) * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 +
              0.21232785429668755 * theta1 + 0.7876721457033125 * theta2).sin() +
          0.0020338944549318037 * delta1.powi(2) * (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 +
              0.045787770837386166 * theta1 + 0.9542122291626138 * theta2).sin() +
          0.00004085411269717685 * delta1.powi(2) * (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 +
              0.001909550754477074 * theta1 + 0.9980904492455229 * theta2).sin());
      jacobians_index = jacobians_index + 1;
      // x_i+1
      jacobians[jacobians_index] = 2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) -
          2.0 * (-x1 + x2 - 0.06474248308443546 * delta1 * (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
              0.998090449245523 * theta1 + 0.0019095507544770018 * theta2).cos() -
              0.13985269574463816 * delta1 * (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
                  0.045787770837386436 * theta2).cos() - 0.1909150252525593 * delta1 *
              (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).cos()
              - 0.20897959183673434 * delta1 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).cos() -
              0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
                  0.7876721457033125 * theta2).cos() - 0.13985269574463816 * delta1 *
              (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
                  0.9542122291626138 * theta2).cos() - 0.06474248308443546 * delta1 *
              (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
                  0.9980904492455229 * theta2).cos());
      jacobians_index = jacobians_index + 1;
      // y_i+1
      jacobians[jacobians_index] = 2.0 * (-y0 + y1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
          0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).sin() -
          0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
              0.045787770837386436 * theta1).sin() - 0.1909150252525593 * delta *
          (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() -
          0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
          0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
              0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
          (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
          - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
          0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) -
          2.0 * (-y1 + y2 - 0.06474248308443546 * delta1 * (0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
              0.998090449245523 * theta1 + 0.0019095507544770018 * theta2).sin() -
              0.13985269574463816 * delta1 * (0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
                  0.045787770837386436 * theta2).sin() - 0.1909150252525593 * delta1 *
              (0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2).sin()
              - 0.20897959183673434 * delta1 * (0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2).sin() -
              0.1909150252525593 * delta1 * (0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
                  0.7876721457033125 * theta2).sin() - 0.13985269574463816 * delta1 *
              (0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
                  0.9542122291626138 * theta2).sin() - 0.06474248308443546 * delta1 *
              (0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
                  0.9980904492455229 * theta2).sin());
      jacobians_index = jacobians_index + 1;
    }
  }
  // println! {"xy_jacobians_index={}", jacobians_index};
  for i in 0..size - 1 {
    let index = i * 4;
    let index1 = (i + 1) * 4;
    let index2 = 4 * size;

    let theta0 = u[index];
    let kappa0 = u[index + 1];
    let x0 = u[index + 2];
    let y0 = u[index + 3];

    let theta1 = u[index1];
    let kappa1 = u[index1 + 1];
    let x1 = u[index1 + 2];
    let y1 = u[index1 + 3];

    let delta = u[index2 + i];

    // delta_i
    jacobians[jacobians_index] = 2.0 * (-0.06474248308443546 * delta * (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
        (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
            0.0019095507544770018 * theta1).cos() - 0.13985269574463816 * delta * (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
        (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos()
        - 0.1909150252525593 * delta * (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
        (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
        0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
        0.1909150252525593 * delta * (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
            (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).cos() -
        0.13985269574463816 * delta * (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
            (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
        - 0.06474248308443546 * delta * (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
        (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
            0.9980904492455229 * theta1).cos() - 0.06474248308443546 *
        (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
            0.0019095507544770018 * theta1).sin() - 0.13985269574463816 *
        (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin()
        - 0.1909150252525593 * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
        0.2123278542966876 * theta1).sin() - 0.20897959183673434 * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
        0.1909150252525593 * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
            0.7876721457033125 * theta1).sin() - 0.13985269574463816 *
        (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
        - 0.06474248308443546 * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
        0.9980904492455229 * theta1).sin()) * (-y0 + y1 - 0.06474248308443546 * delta *
        (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
            0.0019095507544770018 * theta1).sin() - 0.13985269574463816 * delta *
        (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin()
        - 0.1909150252525593 * delta * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
        0.2123278542966876 * theta1).sin() - 0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() -
        0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
            0.7876721457033125 * theta1).sin() - 0.13985269574463816 * delta *
        (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
        - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
        0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).sin()) +
        2.0 * (-x0 + x1 - 0.06474248308443546 * delta * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
            0.998090449245523 * theta0 + 0.0019095507544770018 * theta1).cos() -
            0.13985269574463816 * delta * (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
                0.045787770837386436 * theta1).cos() - 0.1909150252525593 * delta *
            (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).cos() -
            0.20897959183673434 * delta * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
            0.1909150252525593 * delta * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                0.7876721457033125 * theta1).cos() - 0.13985269574463816 * delta *
            (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
            - 0.06474248308443546 * delta * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
            0.001909550754477074 * theta0 + 0.9980904492455229 * theta1).cos()) *
            (-0.06474248308443546 * (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
                0.0019095507544770018 * theta1).cos() - 0.13985269574463816 *
                (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).cos()
                - 0.1909150252525593 * (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
                0.2123278542966876 * theta1).cos() - 0.20897959183673434 * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).cos() -
                0.1909150252525593 * (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
                    0.7876721457033125 * theta1).cos() - 0.13985269574463816 *
                (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).cos()
                - 0.06474248308443546 * (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
                0.9980904492455229 * theta1).cos() + 0.06474248308443546 * delta * (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
                (0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
                    0.0019095507544770018 * theta1).sin() + 0.13985269574463816 * delta * (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
                (0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1).sin()
                + 0.1909150252525593 * delta * (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
                (0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1).sin() +
                0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) * (0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1).sin() +
                0.1909150252525593 * delta * (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
                    (0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1).sin() +
                0.13985269574463816 * delta * (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
                    (0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1).sin()
                + 0.06474248308443546 * delta * (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
                (0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
                    0.9980904492455229 * theta1).sin());
    jacobians_index = jacobians_index + 1;
  }
  // println! {"jacobians_index={}", jacobians_index};
}

// unsafe fn err_x_y_cost(u: &[f64], size: usize) -> f64 {
//   let index2 = 4 * size;
//   let mut cost_value = 0.0;
//   for i in 0..size - 1 {
//     let index = i * 4;
//     let index1 = (i + 1) * 4;
//
//     let theta0 = u[index];
//     let kappa0 = u[index + 1];
//     let x0 = u[index + 2];
//     let y0 = u[index + 3];
//
//     let theta1 = u[index1];
//     let kappa1 = u[index1 + 1];
//     let x1 = u[index1 + 2];
//     let y1 = u[index1 + 3];
//     // println!("x0={}, x1={}, y0={}, y1={}", u[index + 2], u[index1 + 2], u[index + 3], u[index1 + 3]);
//
//     let delta = u[index2 + i];
//
//     let dx = 0.06474248308443546 * delta * cos(
//       0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//           0.0019095507544770018 * theta1) + 0.13985269574463816 * delta * cos(
//       0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//           0.045787770837386436 * theta1) + 0.1909150252525593 * delta * cos(
//       0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//           0.2123278542966876 * theta1) + 0.20897959183673434 * delta * cos(
//       0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//         0.1909150252525593 * delta * cos(
//           0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//         0.13985269574463816 * delta * cos(
//           0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
//         0.06474248308443546 * delta * cos(
//           0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
//     let dy = 0.06474248308443546 * delta * sin(
//       0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//           0.0019095507544770018 * theta1) + 0.13985269574463816 * delta * sin(
//       0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//           0.045787770837386436 * theta1) + 0.1909150252525593 * delta * sin(
//       0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//           0.2123278542966876 * theta1) + 0.20897959183673434 * delta * sin(
//       0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//         0.1909150252525593 * delta * sin(
//           0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//         0.13985269574463816 * delta * sin(
//           0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
//         0.06474248308443546 * delta * sin(
//           0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 + 0.9980904492455229 * theta1);
//
//     cost_value += (x1 - x0 - dx).powf(2.0) + (y1 - y0 - dy).powf(2.0);
//   }
//   cost_value
// }
//
// unsafe fn cost_function_jacobians(u: &[f64], size: usize, jacobians: &mut [f64]) {
//   let mut jacobians_index = 0;
//   let index2 = 4 * size;
//
//   for i in 0..size {
//     if i == 0 {
//       /* start points */
//       let theta0 = u[0];
//       let kappa0 = u[1];
//       let x0 = u[2];
//       let y0 = u[3];
//
//       let theta1 = u[4];
//       let kappa1 = u[5];
//       let x1 = u[6];
//       let y1 = u[7];
//
//       let delta = u[4 * size];
//       // theta0
//       jacobians[jacobians_index] = 2.0 * (-0.06461885402701487 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//           0.0019095507544770018 * theta1) - 0.13344915256089193 * delta *
//           cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//           - 0.15037844758768545 * delta * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//           0.2123278542966876 * theta1) - 0.10448979591836717 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.040536577664873834 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.0064035431837461835 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.00012362905742060293 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//           (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//               (0.06461885402701487 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//                   0.0019095507544770018 * theta1) + 0.13344915256089193 * delta *
//                   sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//                   + 0.15037844758768545 * delta * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//                   0.2123278542966876 * theta1) + 0.10448979591836717 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//                   0.040536577664873834 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                       0.7876721457033125 * theta1) + 0.0064035431837461835 * delta *
//                   sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//                   + 0.00012362905742060293 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//                   0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // kappa0
//       jacobians[jacobians_index] = 2.0 * (-0.0015646651174168634 * pow(delta, 2.0) * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.013704131501108207 * pow(delta, 2.0) * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//               0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
//           0.028023652733535475 * pow(delta, 2.0) * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//               0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.026122448979591793 * pow(delta, 2.0) * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.011843686434051922 * pow(delta, 2.0) * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//               0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
//           0.0020338944549318037 * pow(delta, 2.0) * cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//               0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
//           0.00004085411269717685 * pow(delta, 2.0) * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//           (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//               (0.0015646651174168634 * pow(delta, 2.0) * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//                   0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
//                   0.013704131501108207 * pow(delta, 2.0) * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//                       0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
//                   0.028023652733535475 * pow(delta, 2.0) * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//                       0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
//                   0.026122448979591793 * pow(delta, 2.0) * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//                   0.011843686434051922 * pow(delta, 2.0) * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//                       0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//                   0.0020338944549318037 * pow(delta, 2.0) * sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//                       0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
//                   0.00004085411269717685 * pow(delta, 2.0) * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//                       0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // x0
//       jacobians[jacobians_index] = -2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // y0
//       jacobians[jacobians_index] = -2.0 * (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // println! {"start_jacobians_index={}", jacobians_index};
//     }
//     else if i == size - 2 {
//       /* end points */
//
//       let theta0 = u[index2 - 8];
//       let kappa0 = u[index2 - 7];
//       let x0 = u[index2 - 6];
//       let y0 = u[index2 - 5];
//
//       let theta1 = u[index2 - 4];
//       let kappa1 = u[index2 - 3];
//       let x1 = u[index2 - 2];
//       let y1 = u[index2 - 1];
//
//       let delta = u[5 * size - 2];
//       // println! {"end0_jacobians_index={}", jacobians_index};
//       // theta_i+1
//       jacobians[jacobians_index] = 2.0 * (-0.00012362905742059827 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//           0.0019095507544770018 * theta1) - 0.006403543183746221 * delta *
//           cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//           - 0.04053657766487384 * delta * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//           0.2123278542966876 * theta1) - 0.10448979591836717 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.15037844758768545 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13344915256089196 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06461885402701487 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//           (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//               (0.00012362905742059827 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//                   0.0019095507544770018 * theta1) + 0.006403543183746221 * delta *
//                   sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//                   + 0.04053657766487384 * delta * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//                   0.2123278542966876 * theta1) + 0.10448979591836717 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//                   0.15037844758768545 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                       0.7876721457033125 * theta1) + 0.13344915256089196 * delta *
//                   sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//                   + 0.06461885402701487 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//                   0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // theta_i+1
//       jacobians[jacobians_index] = 2.0 * (0.000040854112697175375 * pow(delta, 2.0) * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
//           0.0020338944549318097 * pow(delta, 2.0) * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//               0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
//           0.01184368643405193 * pow(delta, 2.0) * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//               0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
//           0.026122448979591793 * pow(delta, 2.0) * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//           0.028023652733535482 * pow(delta, 2.0) * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//               0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//           0.013704131501108207 * pow(delta, 2.0) * cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//               0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
//           0.0015646651174168603 * pow(delta, 2.0) * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//           (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//               (-0.000040854112697175375 * pow(delta, 2.0) * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//                   0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//                   0.0020338944549318097 * pow(delta, 2.0) * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//                       0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
//                   0.01184368643405193 * pow(delta, 2.0) * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//                       0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//                   0.026122448979591793 * pow(delta, 2.0) * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//                   0.028023652733535482 * pow(delta, 2.0) * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//                       0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
//                   0.013704131501108207 * pow(delta, 2.0) * sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//                       0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
//                   0.0015646651174168603 * pow(delta, 2.0) * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//                       0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // x_i+1
//       jacobians[jacobians_index] = 2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // y_i+1
//       jacobians[jacobians_index] = 2.0 * (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1));
//       jacobians_index = jacobians_index + 1;
//       // println! {"end1_jacobians_index={}", jacobians_index};
//     }
//     else {
//       let index = (i - 1) * 4;
//       let index1 = i * 4;
//       let index3 = (i + 1) * 4;
//
//       let theta0 = u[index];
//       let kappa0 = u[index + 1];
//       let x0 = u[index + 2];
//       let y0 = u[index + 3];
//
//       let theta1 = u[index1];
//       let kappa1 = u[index1 + 1];
//       let x1 = u[index1 + 2];
//       let y1 = u[index1 + 3];
//
//       let theta2 = u[index3];
//       let kappa2 = u[index3 + 1];
//       let x2 = u[index3 + 2];
//       let y2 = u[index3 + 3];
//
//       let delta = u[index2 + i - 1];
//       let mut delta1 = 0.0;
//       if i <= size - 2 {
//         delta1 = u[index2 + i];
//       } else {
//         delta1 = u[index2 + i - 1];
//       }
//       // theta_i+1
//       jacobians[jacobians_index] = 2.0 * (delta * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1)
//           - 0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
//               0.9542122291626138 * theta1) - 0.06474248308443546 * delta *
//           cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//               0.9980904492455229 * theta1)) * (0.00012362905742059827 *
//           sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//               0.0019095507544770018 * theta1) + 0.006403543183746221 *
//           sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) + 0.04053657766487384 *
//           sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1)
//           + 0.10448979591836717 * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//           0.15037844758768545 * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) + 0.13344915256089196 *
//           sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
//               0.9542122291626138 * theta1) + 0.06461885402701487 *
//           sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//               0.9980904492455229 * theta1)) + delta * (-0.00012362905742059827 *
//           cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//               0.0019095507544770018 * theta1) - 0.006403543183746221 *
//           cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.04053657766487384 *
//           cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1)
//           - 0.10448979591836717 * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.15037844758768545 * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13344915256089196 *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
//               0.9542122291626138 * theta1) - 0.06461885402701487 *
//           cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//               0.9980904492455229 * theta1)) * (-y0 + y1 - 0.06474248308443546 * delta *
//           sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//               0.0019095507544770018 * theta1) - 0.13985269574463816 * delta *
//           sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1)
//           - 0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 +
//               0.9542122291626138 * theta1) - 0.06474248308443546 * delta *
//           sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//               0.9980904492455229 * theta1)) + delta1 * (-x1 + x2 -
//           0.06474248308443546 * delta1 * cos(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
//               0.998090449245523 * theta1 + 0.0019095507544770018 * theta2) -
//           0.13985269574463816 * delta1 * cos(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 +
//               0.9542122291626136 * theta1 + 0.045787770837386436 * theta2) -
//           0.1909150252525593 * delta1 * cos(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
//               0.2123278542966876 * theta2) - 0.20897959183673434 * delta1 *
//           cos(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//           0.1909150252525593 * delta1 * cos(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//               0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//           cos(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//               0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//           cos(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//               0.9980904492455229 * theta2)) * (0.06461885402701487 *
//           sin(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
//               0.0019095507544770018 * theta2) + 0.13344915256089193 *
//           sin(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//               0.045787770837386436 * theta2) + 0.15037844758768545 *
//           sin(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
//               0.2123278542966876 * theta2) + 0.10448979591836717 * sin(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) +
//           0.040536577664873834 * sin(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//               0.7876721457033125 * theta2) + 0.0064035431837461835 *
//           sin(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//               0.9542122291626138 * theta2) + 0.00012362905742060293 *
//           sin(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//               0.9980904492455229 * theta2)) + delta1 * (-0.06461885402701487 *
//           cos(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
//               0.0019095507544770018 * theta2) - 0.13344915256089193 *
//           cos(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//               0.045787770837386436 * theta2) - 0.15037844758768545 *
//           cos(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
//               0.2123278542966876 * theta2) - 0.10448979591836717 * cos(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//           0.040536577664873834 * cos(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//               0.7876721457033125 * theta2) - 0.0064035431837461835 *
//           cos(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//               0.9542122291626138 * theta2) - 0.00012362905742060293 *
//           cos(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//               0.9980904492455229 * theta2)) * (-y1 + y2 - 0.06474248308443546 * delta1 *
//           sin(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
//               0.0019095507544770018 * theta2) - 0.13985269574463816 * delta1 *
//           sin(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//               0.045787770837386436 * theta2) - 0.1909150252525593 * delta1 *
//           sin(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 +
//               0.2123278542966876 * theta2) - 0.20897959183673434 * delta1 *
//           sin(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//           0.1909150252525593 * delta1 * sin(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//               0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//           sin(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//               0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//           sin(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//               0.9980904492455229 * theta2)));
//       jacobians_index = jacobians_index + 1;
//       // kappa_i+1
//       jacobians[jacobians_index] = 2.0 * (0.000040854112697175375 * pow(delta, 2.0) * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) +
//           0.0020338944549318097 * pow(delta, 2.0) * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//               0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) +
//           0.01184368643405193 * pow(delta, 2.0) * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//               0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
//           0.026122448979591793 * pow(delta, 2.0) * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//           0.028023652733535482 * pow(delta, 2.0) * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//               0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//           0.013704131501108207 * pow(delta, 2.0) * cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//               0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) +
//           0.0015646651174168603 * pow(delta, 2.0) * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//           (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//               0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//               0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                   0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//               cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//               0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//               0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                   0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//               cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//               - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//               0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//               (-0.000040854112697175375 * pow(delta, 2.0) * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//                   0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//                   0.0020338944549318097 * pow(delta, 2.0) * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 +
//                       0.9542122291626136 * theta0 + 0.045787770837386436 * theta1) -
//                   0.01184368643405193 * pow(delta, 2.0) * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 +
//                       0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//                   0.026122448979591793 * pow(delta, 2.0) * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//                   0.028023652733535482 * pow(delta, 2.0) * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 +
//                       0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
//                   0.013704131501108207 * pow(delta, 2.0) * sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 +
//                       0.045787770837386166 * theta0 + 0.9542122291626138 * theta1) -
//                   0.0015646651174168603 * pow(delta, 2.0) * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//                       0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//           2.0 * (-0.0015646651174168634 * pow(delta1, 2.0) * cos(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
//               0.998090449245523 * theta1 + 0.0019095507544770018 * theta2) -
//               0.013704131501108207 * pow(delta1, 2.0) * cos(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 +
//                   0.9542122291626136 * theta1 + 0.045787770837386436 * theta2) -
//               0.028023652733535475 * pow(delta1, 2.0) * cos(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 +
//                   0.7876721457033125 * theta1 + 0.2123278542966876 * theta2) -
//               0.026122448979591793 * pow(delta1, 2.0) * cos(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//               0.011843686434051922 * pow(delta1, 2.0) * cos(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 +
//                   0.21232785429668755 * theta1 + 0.7876721457033125 * theta2) -
//               0.0020338944549318037 * pow(delta1, 2.0) * cos(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 +
//                   0.045787770837386166 * theta1 + 0.9542122291626138 * theta2) -
//               0.00004085411269717685 * pow(delta1, 2.0) * cos(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 +
//                   0.001909550754477074 * theta1 + 0.9980904492455229 * theta2)) *
//               (-y1 + y2 - 0.06474248308443546 * delta1 * sin(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
//                   0.998090449245523 * theta1 + 0.0019095507544770018 * theta2) -
//                   0.13985269574463816 * delta1 * sin(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//                       0.045787770837386436 * theta2) - 0.1909150252525593 * delta1 *
//                   sin(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2)
//                   - 0.20897959183673434 * delta1 * sin(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//                   0.1909150252525593 * delta1 * sin(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//                       0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//                   sin(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//                       0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//                   sin(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//                       0.9980904492455229 * theta2)) + 2.0 * (-x1 + x2 - 0.06474248308443546 * delta1 *
//           cos(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
//               0.0019095507544770018 * theta2) - 0.13985269574463816 * delta1 *
//           cos(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//               0.045787770837386436 * theta2) - 0.1909150252525593 * delta1 *
//           cos(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2)
//           - 0.20897959183673434 * delta1 * cos(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//           0.1909150252525593 * delta1 * cos(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//               0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//           cos(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//               0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//           cos(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//               0.9980904492455229 * theta2)) * (0.0015646651174168634 * pow(delta1, 2.0) *
//           sin(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 + 0.998090449245523 * theta1 +
//               0.0019095507544770018 * theta2) + 0.013704131501108207 * pow(delta1, 2.0) *
//           sin(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//               0.045787770837386436 * theta2) + 0.028023652733535475 * pow(delta1, 2.0) *
//           sin(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2)
//           + 0.026122448979591793 * pow(delta1, 2.0) * sin(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) +
//           0.011843686434051922 * pow(delta1, 2.0) * sin(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 +
//               0.21232785429668755 * theta1 + 0.7876721457033125 * theta2) +
//           0.0020338944549318037 * pow(delta1, 2.0) * sin(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 +
//               0.045787770837386166 * theta1 + 0.9542122291626138 * theta2) +
//           0.00004085411269717685 * pow(delta1, 2.0) * sin(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 +
//               0.001909550754477074 * theta1 + 0.9980904492455229 * theta2));
//       jacobians_index = jacobians_index + 1;
//       // x_i+1
//       jacobians[jacobians_index] = 2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) -
//           2.0 * (-x1 + x2 - 0.06474248308443546 * delta1 * cos(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
//               0.998090449245523 * theta1 + 0.0019095507544770018 * theta2) -
//               0.13985269574463816 * delta1 * cos(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//                   0.045787770837386436 * theta2) - 0.1909150252525593 * delta1 *
//               cos(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2)
//               - 0.20897959183673434 * delta1 * cos(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//               0.1909150252525593 * delta1 * cos(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//                   0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//               cos(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//                   0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//               cos(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//                   0.9980904492455229 * theta2));
//       jacobians_index = jacobians_index + 1;
//       // y_i+1
//       jacobians[jacobians_index] = 2.0 * (-y0 + y1 - 0.06474248308443546 * delta * sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//           0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//           0.13985269574463816 * delta * sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//               0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//           sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//           0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//           0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//               0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//           sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//           - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//           0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) -
//           2.0 * (-y1 + y2 - 0.06474248308443546 * delta1 * sin(0.024167517878118265 * delta1 * kappa1 - 0.0006310248039744553 * delta1 * kappa2 +
//               0.998090449245523 * theta1 + 0.0019095507544770018 * theta2) -
//               0.13985269574463816 * delta1 * sin(0.09798975577940272 * delta1 * kappa1 - 0.014543119416486384 * delta1 * kappa2 + 0.9542122291626136 * theta1 +
//                   0.045787770837386436 * theta2) - 0.1909150252525593 * delta1 *
//               sin(0.14678599914523913 * delta1 * kappa1 - 0.062036429130625285 * delta1 * kappa2 + 0.7876721457033125 * theta1 + 0.2123278542966876 * theta2)
//               - 0.20897959183673434 * delta1 * sin(0.125 * delta1 * kappa1 - 0.125 * delta1 * kappa2 + 0.5 * theta1 + 0.5 * theta2) -
//               0.1909150252525593 * delta1 * sin(0.06203642913062524 * delta1 * kappa1 - 0.14678599914523915 * delta1 * kappa2 + 0.21232785429668755 * theta1 +
//                   0.7876721457033125 * theta2) - 0.13985269574463816 * delta1 *
//               sin(0.014543119416486339 * delta1 * kappa1 - 0.09798975577940272 * delta1 * kappa2 + 0.045787770837386166 * theta1 +
//                   0.9542122291626138 * theta2) - 0.06474248308443546 * delta1 *
//               sin(0.0006310248039744781 * delta1 * kappa1 - 0.024167517878118217 * delta1 * kappa2 + 0.001909550754477074 * theta1 +
//                   0.9980904492455229 * theta2));
//       jacobians_index = jacobians_index + 1;
//     }
//   }
//   // println! {"xy_jacobians_index={}", jacobians_index};
//   for i in 0..size - 1 {
//     let index = i * 4;
//     let index1 = (i + 1) * 4;
//     let index2 = 4 * size;
//
//     let theta0 = u[index];
//     let kappa0 = u[index + 1];
//     let x0 = u[index + 2];
//     let y0 = u[index + 3];
//
//     let theta1 = u[index1];
//     let kappa1 = u[index1 + 1];
//     let x1 = u[index1 + 2];
//     let y1 = u[index1 + 3];
//
//     let delta = u[index2 + i];
//
//     // delta_i
//     jacobians[jacobians_index] = 2.0 * (-0.06474248308443546 * delta * (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
//         cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//             0.0019095507544770018 * theta1) - 0.13985269574463816 * delta * (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
//         cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//         - 0.1909150252525593 * delta * (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
//         cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//         0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//         0.1909150252525593 * delta * (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
//             cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) -
//         0.13985269574463816 * delta * (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
//             cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//         - 0.06474248308443546 * delta * (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
//         cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//             0.9980904492455229 * theta1) - 0.06474248308443546 *
//         sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//             0.0019095507544770018 * theta1) - 0.13985269574463816 *
//         sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//         - 0.1909150252525593 * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//         0.2123278542966876 * theta1) - 0.20897959183673434 * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//         0.1909150252525593 * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//             0.7876721457033125 * theta1) - 0.13985269574463816 *
//         sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//         - 0.06474248308443546 * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//         0.9980904492455229 * theta1)) * (-y0 + y1 - 0.06474248308443546 * delta *
//         sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//             0.0019095507544770018 * theta1) - 0.13985269574463816 * delta *
//         sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//         - 0.1909150252525593 * delta * sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//         0.2123278542966876 * theta1) - 0.20897959183673434 * delta * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//         0.1909150252525593 * delta * sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//             0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//         sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//         - 0.06474248308443546 * delta * sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//         0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) +
//         2.0 * (-x0 + x1 - 0.06474248308443546 * delta * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 +
//             0.998090449245523 * theta0 + 0.0019095507544770018 * theta1) -
//             0.13985269574463816 * delta * cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 +
//                 0.045787770837386436 * theta1) - 0.1909150252525593 * delta *
//             cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) -
//             0.20897959183673434 * delta * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//             0.1909150252525593 * delta * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                 0.7876721457033125 * theta1) - 0.13985269574463816 * delta *
//             cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//             - 0.06474248308443546 * delta * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 +
//             0.001909550754477074 * theta0 + 0.9980904492455229 * theta1)) *
//             (-0.06474248308443546 * cos(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//                 0.0019095507544770018 * theta1) - 0.13985269574463816 *
//                 cos(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//                 - 0.1909150252525593 * cos(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 +
//                 0.2123278542966876 * theta1) - 0.20897959183673434 * cos(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) -
//                 0.1909150252525593 * cos(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 +
//                     0.7876721457033125 * theta1) - 0.13985269574463816 *
//                 cos(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//                 - 0.06474248308443546 * cos(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//                 0.9980904492455229 * theta1) + 0.06474248308443546 * delta * (0.024167517878118265 * kappa0 - 0.0006310248039744553 * kappa1) *
//                 sin(0.024167517878118265 * delta * kappa0 - 0.0006310248039744553 * delta * kappa1 + 0.998090449245523 * theta0 +
//                     0.0019095507544770018 * theta1) + 0.13985269574463816 * delta * (0.09798975577940272 * kappa0 - 0.014543119416486384 * kappa1) *
//                 sin(0.09798975577940272 * delta * kappa0 - 0.014543119416486384 * delta * kappa1 + 0.9542122291626136 * theta0 + 0.045787770837386436 * theta1)
//                 + 0.1909150252525593 * delta * (0.14678599914523913 * kappa0 - 0.062036429130625285 * kappa1) *
//                 sin(0.14678599914523913 * delta * kappa0 - 0.062036429130625285 * delta * kappa1 + 0.7876721457033125 * theta0 + 0.2123278542966876 * theta1) +
//                 0.20897959183673434 * delta * (0.125 * kappa0 - 0.125 * kappa1) * sin(0.125 * delta * kappa0 - 0.125 * delta * kappa1 + 0.5 * theta0 + 0.5 * theta1) +
//                 0.1909150252525593 * delta * (0.06203642913062524 * kappa0 - 0.14678599914523915 * kappa1) *
//                     sin(0.06203642913062524 * delta * kappa0 - 0.14678599914523915 * delta * kappa1 + 0.21232785429668755 * theta0 + 0.7876721457033125 * theta1) +
//                 0.13985269574463816 * delta * (0.014543119416486339 * kappa0 - 0.09798975577940272 * kappa1) *
//                     sin(0.014543119416486339 * delta * kappa0 - 0.09798975577940272 * delta * kappa1 + 0.045787770837386166 * theta0 + 0.9542122291626138 * theta1)
//                 + 0.06474248308443546 * delta * (0.0006310248039744781 * kappa0 - 0.024167517878118217 * kappa1) *
//                 sin(0.0006310248039744781 * delta * kappa0 - 0.024167517878118217 * delta * kappa1 + 0.001909550754477074 * theta0 +
//                     0.9980904492455229 * theta1));
//     jacobians_index = jacobians_index + 1;
//   }
//   // println! {"jacobians_index={}", jacobians_index};
// }