#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/range.hpp"


class SonarSimNode : public rclcpp::Node
{

public:

    SonarSimNode()
    : Node("sonar_sim")
    {

        publisher_ =
        this->create_publisher<sensor_msgs::msg::Range>(
            "sonar_measurement",
            10
        );


        subscriber_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "bathymetric_map",
            10,
            std::bind(
                &SonarSimNode::map_callback,
                this,
                std::placeholders::_1
            )
        );


        RCLCPP_INFO(
            this->get_logger(),
            "AUV Sonar Simulator Started"
        );

    }


private:

    void map_callback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg
    )
    {

        auto sonar =
        sensor_msgs::msg::Range();


        sonar.header.frame_id = "sonar";


        sonar.radiation_type =
        sensor_msgs::msg::Range::ULTRASOUND;


        sonar.range =
        msg->data[5000] / 10.0;


        publisher_->publish(sonar);


    }


    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;

};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<SonarSimNode>()
    );

    rclcpp::shutdown();

    return 0;

}

