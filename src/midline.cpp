// ROS 2 versione di midline.cpp per usare GRO
#include "rclcpp/rclcpp.hpp"
#include "GRO.hh" // Assumiamo sia aggiornato per ROS 2

GRO GROobj;

using std::placeholders::_1;
using std::string;

class MidlineNode : public rclcpp::Node {
public:
    MidlineNode() : Node("midline") {
        this->declare_parameter<string>("Paths.midline", "");
        this->declare_parameter<string>("Paths.save", "");

        this->declare_parameter<double>("spacing", 0.05);
        this->declare_parameter<double>("separation", 3.0);
        this->declare_parameter<double>("securityFactor", 2.0);
        this->declare_parameter<bool>("visualization", false);

        std::string midline_path = this->get_parameter("Paths.midline").as_string();
        std::string save_path = this->get_parameter("Paths.save").as_string();
        double spacing = this->get_parameter("spacing").as_double();
        double separation = this->get_parameter("separation").as_double();
        double security = this->get_parameter("securityFactor").as_double();
        bool visualization = this->get_parameter("visualization").as_bool();

        GROobj.midlinePath = midline_path;
        GROobj.savePath = save_path;
        GROobj.spacing = spacing;
        GROobj.separation = separation;
        GROobj.securityFactor = security;
        GROobj.visualization = visualization;

        //path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gro/path", 1);

        auto data = GROobj.read_csv(midline_path);

        RCLCPP_INFO(this->get_logger(), "Midline path: %s", midline_path.c_str());

        GROobj.init(data);
        GROobj.save_data(save_path);

        if (visualization) {
            rclcpp::sleep_for(std::chrono::seconds(2));
            //path_pub_->publish(GROobj.get_path());
            //RCLCPP_WARN(this->get_logger(), "GRO path published in %s", path_pub_->get_topic_name());
        }
    }

private:
    //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<MidlineNode>());
    rclcpp::shutdown();
    return 0;
}