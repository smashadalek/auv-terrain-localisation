#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VehicleSimNode : public rclcpp::Node
{
public:
    VehicleSimNode()
    : Node("vehicle_sim"),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      dt_(0.1)
    {
        pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/simulation/ground_truth_pose",
                10);

        cmd_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                    latest_cmd_ = *msg;
                });

        timer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&VehicleSimNode::update, this));

        RCLCPP_INFO(
            this->get_logger(),
            "AUV vehicle simulator started.");
    }

private:
    void update()
    {
        const double vx = latest_cmd_.linear.x;
        const double vy = latest_cmd_.linear.y;
        const double yaw_rate = latest_cmd_.angular.z;

        x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt_;
        y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt_;
        yaw_ += yaw_rate * dt_;

        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw_ / 2.0);
        pose.pose.orientation.w = std::cos(yaw_ / 2.0);

        pose_pub_->publish(pose);
    }

    double x_;
    double y_;
    double yaw_;
    double dt_;

    geometry_msgs::msg::Twist latest_cmd_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VehicleSimNode>());

    rclcpp::shutdown();

    return 0;
}