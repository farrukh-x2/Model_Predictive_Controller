#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"



using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.


class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  Parameters param;
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs, Parameters param) { this->coeffs = coeffs; this->param = param;}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    
    for (int t = 0; t < param.N; t++) {
      fg[0] += CppAD::pow(vars[param.cte_start + t], 2);
      fg[0] += CppAD::pow(vars[param.epsi_start + t], 2);
      fg[0] += CppAD::pow((vars[param.v_start + t] - param.ref_v), 2);
      //cout<< v_start << " vars[v_start] " << vars[v_start+t] << endl;
    }

    // Minimize the use of actuators.
    for (int t = 0; t < param.N - 1; t++) {
      fg[0] += CppAD::pow(vars[param.delta_start + t], 2);
      fg[0] += CppAD::pow(vars[param.a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < param.N - 2; t++) {
      fg[0] += CppAD::pow(vars[param.delta_start + t + 1] - vars[param.delta_start + t], 2);
      fg[0] += CppAD::pow(vars[param.a_start + t + 1] - vars[param.a_start + t], 2);
    }

    //
    // Setup Constraints
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + param.x_start] = vars[param.x_start];
    fg[1 + param.y_start] = vars[param.y_start];
    fg[1 + param.psi_start] = vars[param.psi_start];
    fg[1 + param.v_start] = vars[param.v_start];
    fg[1 + param.cte_start] = vars[param.cte_start];
    fg[1 + param.epsi_start] = vars[param.epsi_start];

    // The rest of the constraints
    for (int t = 1; t < param.N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[param.x_start + t];
      AD<double> y1 = vars[param.y_start + t];
      AD<double> psi1 = vars[param.psi_start + t];
      AD<double> v1 = vars[param.v_start + t];
      AD<double> cte1 = vars[param.cte_start + t];
      AD<double> epsi1 = vars[param.epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[param.x_start + t - 1];
      AD<double> y0 = vars[param.y_start + t - 1];
      AD<double> psi0 = vars[param.psi_start + t - 1];
      AD<double> v0 = vars[param.v_start + t - 1];
      AD<double> cte0 = vars[param.cte_start + t - 1];
      AD<double> epsi0 = vars[param.epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[param.delta_start + t - 1];
      AD<double> a0 = vars[param.a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + param.x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * param.dt);
      fg[1 + param.y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * param.dt);
      fg[1 + param.psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * param.dt);
      fg[1 + param.v_start + t] = v1 - (v0 + a0 * param.dt);
      fg[1 + param.cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * param.dt));
      fg[1 + param.epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * param.dt);
    }
  }
};


//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}



void MPC::init(size_t N, double dt, double ref_v){


  param.N = N;
  param.dt = dt;
  param.ref_v = ref_v;

  param.x_start = 0;
  param.y_start = param.x_start + param.N;
  param.psi_start = param.y_start + param.N;
  param.v_start = param.psi_start + param.N;
  param.cte_start = param.v_start + param.N;
  param.epsi_start = param.cte_start + param.N;
  param.delta_start = param.epsi_start + param.N;
  param.a_start = param.delta_start + param.N - 1;

}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  //bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = param.N * 6 + (param.N - 1) * 2;
  // Number of constraints
  size_t n_constraints = param.N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
// Set the initial variable values
  vars[param.x_start] = x;
  vars[param.y_start] = y;
  vars[param.psi_start] = psi;
  vars[param.v_start] = v;
  vars[param.cte_start] = cte;
  vars[param.epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < param.delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = param.delta_start; i < param.a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = param.a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }



  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[param.x_start] = x;
  constraints_lowerbound[param.y_start] = y;
  constraints_lowerbound[param.psi_start] = psi;
  constraints_lowerbound[param.v_start] = v;
  constraints_lowerbound[param.cte_start] = cte;
  constraints_lowerbound[param.epsi_start] = epsi;

  constraints_upperbound[param.x_start] = x;
  constraints_upperbound[param.y_start] = y;
  constraints_upperbound[param.psi_start] = psi;
  constraints_upperbound[param.v_start] = v;
  constraints_upperbound[param.cte_start] = cte;
  constraints_upperbound[param.epsi_start] = epsi;
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, param);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> return_vector;

  return_vector.push_back(solution.x[param.delta_start]);
  return_vector.push_back(solution.x[param.a_start]);

  for (int i = 0; i < param.N; ++i) {
      return_vector.push_back(solution.x[param.x_start + i]);
      return_vector.push_back(solution.x[param.y_start + i]);
  }

  return return_vector;


}
