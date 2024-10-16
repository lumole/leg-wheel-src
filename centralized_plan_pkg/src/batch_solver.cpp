#include "batch_solver.h"

void BatchSolver::set_initial_states(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& theta) {
    xinit_ = x;
    yinit_ = y;
    thetainit_ = theta;
};
void BatchSolver::set_ref_states(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& theta) {
    xr_ = x;
    yr_ = y;
    thetar_ = theta;
};
void BatchSolver::set_ref_states(const std::vector<double>& v) {
    vr_ = v;
};
void BatchSolver::set_obst_(const std::vector<std::vector<double>>& obst) {
    obst_ = obst;
};


bool BatchSolver::Solve(std::vector<std::vector<std::vector<double>>>& pre_states, std::vector<std::vector<std::vector<double>>>& pre_inputs) {

    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t num_states = (5 * (N_ - 1) + 10) * m_ + (4 * (N_ - 1) + 8) * (m_ * (m_ - 1) / 2 + m_ * obst_.size());
    Dvector xi(num_states);
    if (warmstart_.empty()) {
        for (size_t i = 0; i < num_states; i++) {
            xi[i] = 0.0;
        }
    }
    else {
        for (size_t i = 0; i < num_states; i++) {
            xi[i] = warmstart_[i];
        }
    }

    //std::cout<<xi<<std::endl;
    // lower and upper limits for x
    Dvector xl(num_states), xu(num_states);
    size_t step_x = 5 * (N_ - 1) + 10;
    for (size_t i = 0; i < m_; i++) {
        for (size_t j = 0; j < N_ + 1; j++) {
            xl[i * step_x + j * 5] = -1000.0;xu[i * step_x + j * 5] = 1000.0;
            xl[i * step_x + j * 5 + 1] = -1000.0;xu[i * step_x + j * 5 + 1] = 1000.0;//xy限制
            // xl[i * step_x + j * 5 + 2] = -4.0 * 3.14;xu[i * step_x + j * 5 + 2] = 4.0 * 3.14;//转角限制
            xl[i * step_x + j * 5 + 2] = -1.0e19;xu[i * step_x + j * 5 + 2] = 1.0e19;
            // xl[i * step_x + j * 5 + 3] = -0.5;xu[i * step_x + j * 5 + 3] = 0.5;//速度限制
            xl[i * step_x + j * 5 + 3] = -vr_[i];xu[i * step_x + j * 5 + 3] = vr_[i];//速度限制
            // xl[i * step_x + j * 5 + 4] = -3.14 / 3.0;xu[i * step_x + j * 5 + 4] = 3.14 / 3.0;//角速度限制
            xl[i * step_x + j * 5 + 4] = -3.14 / 3.0;xu[i * step_x + j * 5 + 4] = 3.14 / 3.0;
        }

    }

    size_t var = (5 * (N_ - 1) + 10) * m_;
    size_t start_i = var; // equal to var1 above
    for (size_t i = 0; i < m_ - 1;i++) {
        if (i > 0) start_i += (4 * (N_ - 1) + 8) * (m_ - i); // equal to var2 3 4 5 ...(m-1) above
        for (size_t j = 0; j < m_ - i - 1;j++) {
            size_t start_j = start_i + j * (4 * (N_ - 1) + 8);
            for (size_t k = 0; k < N_ + 1;k++) {
                xl[start_j + k * 4] = -1.0e19;xu[start_j + k * 4] = 1.0e19;
                xl[start_j + k * 4 + 1] = -1.0e19;xu[start_j + k * 4 + 1] = 1.0e19;
                xl[start_j + k * 4 + 2] = -1.0e19;xu[start_j + k * 4 + 2] = 1.0e19;
                xl[start_j + k * 4 + 3] = 0.0;xu[start_j + k * 4 + 3] = 1.0e19;

            }
        }
    }

    size_t start_obst = (5 * (N_ - 1) + 10) * m_ + (4 * (N_ - 1) + 8) * m_ * (m_ - 1) / 2;
    for (size_t i = 0; i < m_;i++) {
        for (size_t j = 0; j < obst_.size();j++) {
            for (size_t k = 0; k < N_ + 1;k++) {
                xl[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4] = -1.0e19;xu[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4] = 1.0e19;
                xl[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 1] = -1.0e19;xu[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 1] = 1.0e19;
                xl[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 2] = -1.0e19;xu[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 2] = 1.0e19;
                xl[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 3] = 0.0;xu[start_obst + i * obst_.size() * (N_ + 1) * 4 + j * 4 * (N_ + 1) + k * 4 + 3] = 1.0e19;
            }
        }

    }


    //std::cout<<xl<<std::endl;
    //std::cout<<xu<<std::endl;
    // lower and upper limits for g
    size_t size_tmp = 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 3 + 3 * m_ + 3 * (N_ + 1) * obst_.size() * m_;

    Dvector gl(size_tmp), gu(size_tmp);
    for (size_t i = 0; i < 3 * N_ * m_; i++) {// kinematics
        gl[i] = 0.0;     gu[i] = 0.0;
    }



    for (size_t i = 3 * N_ * m_; i < 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 3; i++) {// inter-vehicles  avoidance
        gl[i] = 0.0;     gu[i] = 1.0e19;
    }



    for (size_t i = 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 4; i < 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 3 + 3 * m_; i++) {// intial constraints
        gl[i] = 0.0;     gu[i] = 0.0;
    }

    for (size_t i = 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 3 + 3 * m_; i < 3 * N_ * m_ + 3 * (N_ + 1) * (m_ + 1) * (m_ - 2) / 2.0 + 3 * N_ + 3 + 3 * m_ + 3 * (N_ + 1) * obst_.size() * m_; i++) {// obst-vehicles constraints
        gl[i] = 0.0;     gu[i] = 1.0e19;
    }



    //std::cout<<3*N_*(m_+(m_+1)*(m_-2)/2+1)+3<<"jijijkjk "<<std::endl;
     //  std::cout<<gl<<std::endl;
     //  std::cout<<gu<<std::endl;
        // object that computes objective and constraints
    //exit(0);


        // N_, m_, xr_, yr_, thetar_, d_, xinit_,yinit_, thetainit_, ts_, safety_dist_
    FG_eval fg_eval(N_, m_, xr_, yr_, thetar_, vr_, d_, xinit_, yinit_, thetainit_, ts_, safety_dist_, obst_);

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    //options += "Retape  true\n";
    options += "Sparse true        forward\n";
    options += "Sparse true         reverse\n";
    // options += "Numeric max_cpu_time          0.5\n";


     // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    auto start_t = std::chrono::system_clock::now();

    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_t);
    std::cout << "Time spent: "
        << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "seconds";
    //
    // Check some of the solution values
    //
    std::cout << "state : " << solution.status << std::endl;
    std::cout << "obj : " << solution.obj_value << std::endl;
    //
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (ok) {
        size_t step_1 = 5 * (N_ - 1) + 10;
        for (size_t i = 0; i < m_;i++) {
            for (size_t j = 0; j < N_ + 1;j++) {
                pre_states[i][j][0] = solution.x[i * step_1 + j * 5];
                pre_states[i][j][1] = solution.x[i * step_1 + j * 5 + 1];
                pre_states[i][j][2] = solution.x[i * step_1 + j * 5 + 2];
                pre_inputs[i][j][0] = solution.x[i * step_1 + j * 5 + 3];
                pre_inputs[i][j][1] = solution.x[i * step_1 + j * 5 + 4];
                //std::cout<<" ssss: "<<pre_states[i][j][0]<<" "<<pre_states[i][j][1]<<" "<<pre_states[i][j][2]<<std::endl;


            }

        };
        warmstart_.clear();
        for (size_t i = 0; i < (5 * (N_ - 1) + 10) * m_ + (4 * (N_ - 1) + 8) * (m_ * (m_ - 1) / 2 + m_ * obst_.size());i++) {

            warmstart_.push_back(solution.x[i]);

        }

    }



    return ok;


}
