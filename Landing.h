//
// Created by han on 2020/12/26.
//

#ifndef CASSIE_JUMP_PART2_LANDING_H
#define CASSIE_JUMP_PART2_LANDING_H


#include <memory>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/solve.h>

#include "OptNode.h"

class Landing {
public:
    Landing(int var_num_per_node, int node_num, double dt);
    Landing(Landing& ) = delete;
    Landing(Landing&& ) = delete;

    void build();

    void print_var(const drake::solvers::MathematicalProgramResult& result);

    std::unique_ptr<Eigen::VectorXd> var_sol_;
    std::shared_ptr<drake::solvers::MathematicalProgram> program_;

private:
    int var_num_per_node_;
    int node_num_;
    double dt_;

    std::vector<std::unique_ptr<OptNode>> node_list_;

    std::vector<drake::solvers::Constraint*> direct_collocation_constraints_;

    std::vector<drake::solvers::LinearConstraint*> kinematics_constraints_;

};


#endif //CASSIE_JUMP_PART2_LANDING_H
