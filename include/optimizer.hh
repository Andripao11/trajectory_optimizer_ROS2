/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OPTIMIZER_HH
#define OPTIMIZER_HH

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <eigen3/Eigen/Dense>
#include <cerrno>

#include "casadi/casadi.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp" //per read_csv path

using namespace casadi;
using namespace std;
using namespace Eigen;

// Non Linear Optimization Problem (NLOP) struct
struct NLOProblem {

  int N; // Horizon length

  string flag; // Solver flag

  vector<double> lbx;        // Lower boundaries stages (x,u)
  vector<double> ubx;        // Upper boundaries stages (x,u)

  vector<double> lbg_next;   // Lower boundaries continuity constraints 
  vector<double> ubg_next;   // Upper boundaries continuity constraints

  vector<double> lbg_track;  // Lower boundaries track constraints
  vector<double> ubg_track;  // Upper boundaries track constraints

  vector<double> lbg_elipse;  // Lower boundaries ellipse constraints + initial==final (last constraint)
  vector<double> ubg_elipse;  // Upper boundaries ellipse constraints + initial==final (last constraint)

  vector<double> x0;         // Initial guess
  vector<double> p;          // Parameters + curvature
  vector<double> solution;   // Solution --> Optimized stages

};

// TRO class
class optimizer{

  rclcpp::Logger logger_{rclcpp::get_logger("optimizer")};

    private:

        // Internal variables/methods of TRO

        int n_states = 7; // Number of states variables [delta, acc, n, mu, Vx, Vy, w]
        int n_controls = 2; // Number of controls variables [diffDelta, diffAcc] = [d(delta)/ds, d(acc)/ds]
        int N; // Horizon length for optimization problem 
        int Npar; // Number of parameters for optimization problem [ 23 (MPC parameters) + 7 (initial state) + n (curvature points == N) ]

        double T; // discretization of TRO
        double dt = 0.05; // discretization of GRO

        double lower_continuity = -0.01; // Lower bound continuity constraint
        double upper_continuity = 0.01;  // Upper bound continuity constraint

        double lower_track = 0.0;        // Lower bound track constraint
        double upper_track = 1.5;        // Upper bound track constraint

        double lower_ellipse = -inf;       // Lower bound ellipse constraint 
        double upper_ellipse = 0.0;      // Upper bound ellipse constraint

        bool isRun = false; // Flag set to true if the solver is working

        void boundaries(); // Set boundaries for the NLOP
        void parameters(); // Set parameters for the NLOP
        void Solve(); // Set equality/inequality constraints --> Set objective function --> Solve the NLOP

        vector<SX> continuous_dynamics( SX st, SX con, SX P, SX k);
        vector<double> vconcat(const vector<double>& x, const vector<double>& y);
        void printVec(vector<double> &input, int firstElements=0);

        // Read & Write csv data
        MatrixXd read_csv(string& filename, bool longvec=false, bool firstout=false);
        void save_csv(string& filePath, string name, vector<double>& data);
        void save_info(string& filePath, string name);

        // VARIABLES BOUNDARIES:

          // Bounds and initial guess for the control
        vector<double> u_min =  { -5*M_PI/180, -5.0 };
        vector<double> u_max  = {  5*M_PI/180, 0.5  };
        vector<double> u0 = {  0.0, 0.0  };

          // Bounds and initial guess for the state
        vector<double> x_min  = { -25.0*M_PI/180, -12.0, -2, -100.0*M_PI/180, 0.0, -2.0, -100.0*M_PI/180 };
        vector<double> x_max  = { 25.0*M_PI/180, 15, 2, 100.0*M_PI/180, 25.0, 2.0, 100.0*M_PI/180 };
        vector<double> x0 = { 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0 };

          // Bounds initial==final constraint
        vector<double> lower_final = { -0.01, -0.1, -0.1, -0.5, -0.5 };         // { -0.1, -0.1, -0.1, -0.5, -5*M_PI/180 };
        vector<double> upper_final = { 0.01, 0.1, 0.1, 0.5, 0.5 };              //{ 0.1, 0.1, 0.1, 0.5, 5*M_PI/180 };


    public:

        optimizer(); // Constructor

        bool debugging = false; // if true debug function is called

        // Read path
        std::string curvaturePath; // only input of TROsolver  

        // Save paths
        string savePath = "/data/"; // path where output is saved

        void init(); // Initialization function
        bool ExitFlag(); // Return true if Solve Succeded
        bool isRunning(); // Return isRun
        void debug(); // Debug info 

        // NLOP data
        NLOProblem nlop = NLOProblem();

        // CAR PARAMETERS:
        double longue = 2.63;
        double width = 1.47;

        // PARAMETERS:
        double dRd = 1;
        double dRa = 0.3; 
        double m = 230;
        double I = 140;
        double Lf = 0.800;
        double Lr = 0.735;
        double Dr = 3152.3;
        double Df = 2785.4;
        double Cr = 1.6;
        double Cf = 1.6;
        double Br = 10.1507;
        double Bf = 10.8529;
        double u_r = 0.4; //Resistenza al rotolamento
        double gravity = 9.81;
        double Cd = 1.13583;
        double rho = 1.255;
        double Ar = 1.04;
        double q_slip = 0.1;
        double p_long = 0.5;
        double q_n = 0.1;
        double q_mu = 0.1;
        double lambda = 1;
        double q_s = 1;

        // Discretization of middle trajectory's curvature
        int curvatureStride = 10;

        int horizonLength = 0; // if > 0 it sets the horizon length of the NLOP --> curvatureStride won't be used

        // Solver options --> for IPOPT solver options, see http://casadi.sourceforge.net/v2.0.0/api/html/d6/d07/classcasadi_1_1NlpSolver.html#plugin_NlpSolver_ipopt
        //                                                  https://coin-or.github.io/Ipopt/OPTIONS.html

        Dict ipoptOptions = {{"max_iter",5000},{"expect_infeasible_problem","yes"},{"print_level", 5}}; 
        Dict solverOptions = {{"ipopt",ipoptOptions},{"print_time",true}};

};

#endif