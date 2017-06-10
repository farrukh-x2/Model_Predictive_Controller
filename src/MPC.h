#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
struct Parameters {
    // reference values

    size_t N;
    double dt;
    double ref_v;
    double max_a;

    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t a_start;
};

class MPC {
 public:
  Parameters param;

  MPC();

  virtual ~MPC();

  void init(size_t N, double dt, double ref_v);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
