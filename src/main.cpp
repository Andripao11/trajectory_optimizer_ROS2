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

#include "rclcpp/rclcpp.hpp"
#include "../include/TRO.hh"

// TRO is initialized here
TRO TROobj;

/*
// This is the planner callback
void callback_planner(const slam_msgs::msg::PositionCarLandmark::SharedPtr data){

    // Don't do anything until the TRO is initialized
    if (TROobj.isRunning()){
        
        if(!TROobj.firstSpFlag) TROobj.join_track(data);
        path_interfaces::msg::ObjectiveArrayCurv msgCurv = TROobj.plannerTRO(data);
        troPlanner->publish(msgCurv);

        // Visualization of the path
        //troPath->publish(TROobj.get_path());
        //troFirstPath->publish(TROobj.get_Firstpath());
        //TROobj.pubMarkerArray(TROobj.read_cones(TROobj.conesPath), pubConesLoopMA);
    }
}
*/


int main(int argc, char **argv){

    // Init Node:
    rclcpp::init(argc, argv);

    // Handle Connections:
    auto node = rclcpp::Node::make_shared("tro");

    // Visualization publishers
    TROobj.pubPath = node->create_publisher<nav_msgs::msg::Path>("tro/optimized_path", 1);
    TROobj.pubCones = node->create_publisher<visualization_msgs::msg::MarkerArray>("tro/cones", 1);

    // Topics
    string plannerTopic, pathTopic, FirstpathTopic, conesTopic, stateCarTopic;

    // Setting params from yaml
    TROobj.optimizedPath     = node->declare_parameter<std::string>("Paths.Optimized",     "");
    TROobj.problemPath       = node->declare_parameter<std::string>("Paths.Problem",       "");
    
    RCLCPP_INFO(
    node->get_logger(),
    "DEBUG – problemPath letto dal parametro: %s",
    TROobj.problemPath.c_str());
    
    TROobj.middlePointsPath  = node->declare_parameter<std::string>("Paths.MiddlePoints",  "");
    TROobj.freeSpacePath     = node->declare_parameter<std::string>("Paths.FreeSpace",     "");
    TROobj.savePath          = node->declare_parameter<std::string>("Paths.Save",          "");

    TROobj.splinesStride     = node->declare_parameter<int>("splinesStride",  12);
    TROobj.firstSpId         = node->declare_parameter<int>("firstSpId",      0);

    TROobj.Tmiddle           = node->declare_parameter<double>("Tmiddle",     0.05);
    TROobj.v0                = node->declare_parameter<double>("v0",          0.0);

    TROobj.conesPath         = node->declare_parameter<std::string>("conesPath", "");

    // Initialization of TRO
    if(not TROobj.isRunning()){
        // Initialization
        TROobj.init();
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Traiettoria ha " << TROobj.traj.pointsTraj.rows() << " punti\n";
    TROobj.publish_visualization(node);

    std::cout << "✔️  Traiettoria e bordi pubblicati con successo.\n";

    cout << "HO FINITOOOOOOO!!!" <<endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

