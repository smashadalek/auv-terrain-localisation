#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class VisualisationNode : public rclcpp::Node
{
public:
    VisualisationNode()
    : Node("auv_visualisation")
    {
        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::Marker>(
                "/visualisation/ground_truth_marker",
                10);

        pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/simulation/ground_truth_pose",
                10,
                std::bind(
                    &VisualisationNode::pose_callback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "AUV visualisation node started.");
    }

private:
    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        visualization_msgs::msg::Marker marker;

        marker.header = msg->header;

        marker.ns = "auv_ground_truth";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = msg->pose;

        // Arrow dimensions
        marker.scale.x = 2.0;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;

        // Temporary debug colour
        marker.color.r = 1.0;
        marker.color.g = 0.4;
        marker.color.b = 0.1;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VisualisationNode>());

    rclcpp::shutdown();

    return 0;
}