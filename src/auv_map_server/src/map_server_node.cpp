#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


class MapServerNode : public rclcpp::Node
{

public:

    MapServerNode()
    : Node("map_server")
    {

        publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "bathymetric_map",
            10
        );


        publish_map();


        RCLCPP_INFO(
            this->get_logger(),
            "AUV Bathymetric Map Server Started"
        );

    }


private:


    void publish_map()
    {

        auto map =
        nav_msgs::msg::OccupancyGrid();


        map.header.frame_id = "map";


        map.info.resolution = 1.0;
        map.info.width = 100;
        map.info.height = 100;


        map.info.origin.position.x = 0;
        map.info.origin.position.y = 0;


        map.data.resize(
            map.info.width *
            map.info.height
        );


        for(size_t i = 0; i < map.data.size(); i++)
        {

            // temporary synthetic terrain
            map.data[i] = i % 100;

        }


        publisher_->publish(map);

    }


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

};



int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<MapServerNode>()
    );

    rclcpp::shutdown();

    return 0;

}
