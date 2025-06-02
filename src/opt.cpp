/*
 Copyright (c) 2023 Oriol Martínez @fetty31

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as ed by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "../include/optimizer.hh"

optimizer OPTobj;

int main(int argc, char **argv){

    // Init Node:
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("optimizer");

    // Setting params from yaml
    // 1. leggo i parametri PRIMA di init()
    OPTobj.curvaturePath = node->declare_parameter<std::string>("Paths.curvature", "");

    OPTobj.savePath = node->declare_parameter<std::string>("Paths.save", "");

    OPTobj.debugging = node->declare_parameter<bool>("debugging", false);

    OPTobj.horizonLength    = node->declare_parameter<int>("horizonLength", 0);
    OPTobj.curvatureStride  = node->declare_parameter<int>("curvatureStride", 10);

    OPTobj.dRd      = node->declare_parameter<double>("dRd");
    OPTobj.dRa      = node->declare_parameter<double>("dRa");
    OPTobj.m        = node->declare_parameter<double>("m");
    OPTobj.I        = node->declare_parameter<double>("Iz");
    OPTobj.Lf       = node->declare_parameter<double>("Lf");
    OPTobj.Lr       = node->declare_parameter<double>("Lr");
    OPTobj.Dr       = node->declare_parameter<double>("Dr");
    OPTobj.Df       = node->declare_parameter<double>("Df");
    OPTobj.Cr       = node->declare_parameter<double>("Cr");
    OPTobj.Cf       = node->declare_parameter<double>("Cf");
    OPTobj.Br       = node->declare_parameter<double>("Br");
    OPTobj.Bf       = node->declare_parameter<double>("Bf");
    OPTobj.u_r      = node->declare_parameter<double>("u_r");
    OPTobj.gravity  = node->declare_parameter<double>("gravity");
    OPTobj.Cd       = node->declare_parameter<double>("Cd");
    OPTobj.rho      = node->declare_parameter<double>("rho");
    OPTobj.Ar       = node->declare_parameter<double>("Ar");
    OPTobj.q_slip   = node->declare_parameter<double>("q_slip");
    OPTobj.p_long   = node->declare_parameter<double>("p_long");
    OPTobj.q_n      = node->declare_parameter<double>("q_n");
    OPTobj.q_mu     = node->declare_parameter<double>("q_mu");
    OPTobj.lambda   = node->declare_parameter<double>("lambda");
    OPTobj.q_s      = node->declare_parameter<double>("q_s");

    /*
    RCLCPP_INFO(node->get_logger(), "============== PARAMETRI OPTIMIZER ==============");
    RCLCPP_INFO(node->get_logger(), "Paths:");
    RCLCPP_INFO(node->get_logger(), "  curvaturePath = %s", OPTobj.curvaturePath.c_str());
    RCLCPP_INFO(node->get_logger(), "  savePath      = %s", OPTobj.savePath.c_str());

    RCLCPP_INFO(node->get_logger(), "Parametri:");
    RCLCPP_INFO(node->get_logger(), "  dRd     = %f", OPTobj.dRd);
    RCLCPP_INFO(node->get_logger(), "  dRa     = %f", OPTobj.dRa);
    RCLCPP_INFO(node->get_logger(), "  m       = %f", OPTobj.m);
    RCLCPP_INFO(node->get_logger(), "  Iz      = %f", OPTobj.I);
    RCLCPP_INFO(node->get_logger(), "  Lf      = %f", OPTobj.Lf);
    RCLCPP_INFO(node->get_logger(), "  Lr      = %f", OPTobj.Lr);
    RCLCPP_INFO(node->get_logger(), "  Dr      = %f", OPTobj.Dr);
    RCLCPP_INFO(node->get_logger(), "  Df      = %f", OPTobj.Df);
    RCLCPP_INFO(node->get_logger(), "  Cr      = %f", OPTobj.Cr);
    RCLCPP_INFO(node->get_logger(), "  Cf      = %f", OPTobj.Cf);
    RCLCPP_INFO(node->get_logger(), "  Br      = %f", OPTobj.Br);
    RCLCPP_INFO(node->get_logger(), "  Bf      = %f", OPTobj.Bf);
    RCLCPP_INFO(node->get_logger(), "  u_r     = %f", OPTobj.u_r);
    RCLCPP_INFO(node->get_logger(), "  gravity = %f", OPTobj.gravity);
    RCLCPP_INFO(node->get_logger(), "  Cd      = %f", OPTobj.Cd);
    RCLCPP_INFO(node->get_logger(), "  rho     = %f", OPTobj.rho);
    RCLCPP_INFO(node->get_logger(), "  Ar      = %f", OPTobj.Ar);
    RCLCPP_INFO(node->get_logger(), "  q_slip  = %f", OPTobj.q_slip);
    RCLCPP_INFO(node->get_logger(), "  p_long  = %f", OPTobj.p_long);
    RCLCPP_INFO(node->get_logger(), "  q_n     = %f", OPTobj.q_n);
    RCLCPP_INFO(node->get_logger(), "  q_mu    = %f", OPTobj.q_mu);
    RCLCPP_INFO(node->get_logger(), "  lambda  = %f", OPTobj.lambda);
    RCLCPP_INFO(node->get_logger(), "  q_s     = %f", OPTobj.q_s);
    RCLCPP_INFO(node->get_logger(), "Discretizzazione:");
    RCLCPP_INFO(node->get_logger(), "  horizonLength    = %d", OPTobj.horizonLength);
    RCLCPP_INFO(node->get_logger(), "  curvatureStride  = %d", OPTobj.curvatureStride);
    RCLCPP_INFO(node->get_logger(), "=================================================");
*/


    // Initialization of OPTIMIZER
    if(not OPTobj.isRunning()) OPTobj.init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}