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

#ifndef GRO_HH
#define GRO_HH

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
//#include "nav_msgs/msg/path.hpp"
//#include "trajectory_optimizer/msg/tracklimits.hpp"
//#include "trajectory_optimizer/msg/objective_array_curv.hpp"
//#include "trajectory_optimizer/msg/objective_curv.hpp"
//#include "trajectory_optimizer/msg/car_state.hpp"

#include "kdtree.h"

// --- tipi “finti” per lavorare offline, giusto quello che serve ---
namespace as_msgs
{
  struct Cone
  {
    struct Position { double x, y; } position_global;
  };

  struct Tracklimits
  {
    std::vector<Cone> left;
    std::vector<Cone> right;
  };
} // namespace as_msgs

using namespace std;
using namespace Eigen;

class Point : public std::array<float,2> {
public:
    static const int DIM = 2;
};

struct trajectory {
    MatrixXd Pleft, Pright;
    int N;
    MatrixXd pointsSol, coefsSplinesTrajX, coefsSplinesTrajY;
    VectorXd splinesLengths;
    MatrixXd pointsTraj2;
    VectorXd radiCurv2;
    VectorXd radiCurv;
    VectorXd freeL, freeR;
    MatrixXd pointsTraj;
    VectorXd velocity;
    kdt::KDTree<Point> trajTree;
};

class GRO {
private:
    double dt = 0.05;
    double axmaxAccel = 4.0;
    double axmaxDecel = 8.0;
    double aymax = 15.0;
    double distBound = 2;
    bool isRun = false;
    const static int dimSolver = 500;
    bool solverFlag = false;
    double carWidth = 1.36;

    void get_data(as_msgs::Tracklimits &data);
    void get_trajectory();
    void radi_curv();
    void velocity_profile();
    void create_KDTree();

    void gate_generator(as_msgs::Tracklimits &data);
    MatrixXd coefs_splines(VectorXd x);
    MatrixXd matrix_D(int N);

    Vector2d get_gate_point(Vector2d  P, Vector2d  Q, Vector2d  O, Vector2d  A, Vector2d  B, Vector4d coefsX, Vector4d coefsY);
    void reduce_points();
    VectorXd polyval(Vector4d coeffs, VectorXd t);
    double polyval2(Vector4d coeffs, double t);
    double integral_length(Vector4d coefsX, Vector4d coefsY);
    double f_accel(int k, double v);
    double f_decel(int k, double v);

    //MatrixXd planning_curv(const trajectory_optimizer::msg::CarState::SharedPtr &data);
    int steps = 60;

public:
    GRO();
    void init(as_msgs::Tracklimits &data);
    bool isRunning();
    /** Increase the allowed accelerations (used only for interactive tests). */
    void enjoy();
    //nav_msgs::msg::Path get_path();
    //trajectory_optimizer::msg::ObjectiveArrayCurv plannerGRO_curv(const trajectory_optimizer::msg::CarState::SharedPtr &data);

    as_msgs::Tracklimits read_csv(const std::string &filename);
    void save_data(std::string filename);

    trajectory traj;

    std::string midlinePath;
    std::string savePath;

    double securityFactor = 2;
    double spacing;
    double separation = 4;
    bool visualization;
};

#endif