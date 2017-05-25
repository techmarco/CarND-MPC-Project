#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Timestep length and duration
static double dt = 0.1;
static size_t N = 8;

// Solver variable positions
static size_t x_start = 0;
static size_t y_start = x_start + N;
static size_t psi_start = y_start + N;
static size_t v_start = psi_start + N;
static size_t cte_start = v_start + N;
static size_t epsi_start = cte_start + N;
static size_t delta_start = epsi_start + N;
static size_t a_start = delta_start + N - 1;  

class MPC {
 public:

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  CppAD::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
