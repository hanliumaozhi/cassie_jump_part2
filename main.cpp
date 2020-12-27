#include <iostream>
#include <chrono>

#include "Landing.h"

int main() {
  Landing Landing(9, 51, 0.02);

  Landing.build();

  auto ss = drake::solvers::SnoptSolver();

  Landing.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                                    "mpc_snopt.out");
  //mpc.program_->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
  //                              1);
  //auto ii = drake::solvers::IpoptSolver();


  auto t1 = std::chrono::high_resolution_clock::now();
  const drake::solvers::MathematicalProgramResult result = ss.Solve(*Landing.program_);
  auto t2 = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

  std::cout << duration<<std::endl;

  std::cout<<result.is_success()<<std::endl;
  Landing.print_var(result);
  return 0;
  return 0;
}
