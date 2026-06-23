#include "rclcpp/rclcpp.hpp"


class ParticleFilterNode : public rclcpp::Node
{

public:

    ParticleFilterNode()
    : Node("particle_filter")
    {
        RCLCPP_INFO(
            this->get_logger(),
            "AUV Particle Filter Node Started"
        );
    }

};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ParticleFilterNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
