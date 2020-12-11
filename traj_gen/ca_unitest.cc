#include <cstdio>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <list>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

using namespace std;
using namespace Eigen;
using namespace drake;

int main(){
    solvers::MathematicalProgram prog = solvers::MathematicalProgram();

    // Instantiate the decision variables
    // solvers::VectorXDecisionVariable x = prog.NewContinuousVariables(2, "x");
    auto x = prog.NewContinuousVariables(2, 20, "x");
    auto x0 = MatrixXd(2, 20);
    x0.setZero();

    // Define the cost
    // MatrixXd Q(2,2);
    // MatrixXd new_Q(2,2);
    // Q << 2, 1,
    //      1, 4;
    // new_Q << 3, 1,
    //      1, 4;
    // cout << Q << endl;
    // VectorXd b(2);
    // b << 0, 3;
    // double c = 1.0;
    
    for(unsigned int i=0;i<20;i++){
        auto x_var = x.col(i);
        // auto cost = prog.AddQuadraticCost(Q, b, c, x_var);
        auto cost2 = prog.AddL2NormCost(Matrix2d::Identity(2,2), Vector2d::Ones(2), x_var);
        prog.AddConstraint(pow(x_var[0],2) <= 0.4);
    }
    
    // prog.AddConstraint(pow(x[0],2) <= 0.4);
    // solvers::GurobiSolver solver;
    // cout << "Gurobi available? " << solver.is_enabled() << endl;
//     solvers::ScsSolver solver;
//     prog.SetSolverOption(solver.id(), "Method", 2);

    // auto result = solver.Solve(prog);
    prog.SetInitialGuess(x, x0);
    auto result = solvers::Solve(prog);
    cout << "Is optimization successful?" << result.is_success() << endl;
    cout << "Optimal x: " << result.GetSolution().transpose() << endl;
    cout << "solver is: " << result.get_solver_id().name() << endl;
    // cout << "computation time is: " << result.get_solver_details<solvers::GurobiSolver>().optimizer_time;
//     cost.evaluator()->UpdateCoefficients(new_Q, b, c);
//     result = solvers::Solve(prog, Vector2d::Zero());
//     cout << "Is optimization successful?" << result.is_success() << endl;
//     cout << "Optimal x: " << result.GetSolution().transpose() << endl;
    return 0;
}