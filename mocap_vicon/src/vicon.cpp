#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mocap_vicon/ViconDriver.h>

using namespace std::chrono_literals;

class ViconNode : public rclcpp::Node
{
    public:
        ViconNode() : Node("vicon_node")
        {
            // Parameters are declared and read inside driver.init(*this), which
            // uses this node's parameter server so that launch-file overrides
            // (e.g. frame_rate, server_address) are correctly applied.
            if(!driver.init(*this))
            {
                RCLCPP_ERROR(this->get_logger(), "Initialization of the Vicon driver failed");
                exit(1);
            }
            RCLCPP_INFO(this->get_logger(), "Successfully initialize Vicon connection!");
            
	    while (rclcpp::ok())
	    {
		    driver.run();
		    //RCLCPP_INFO(this->get_logger(), "Running");
	    }

	    RCLCPP_INFO(this->get_logger(), "Shutting Down Vicon");
	    driver.disconnect();
	    
	    // timer = this->create_wall_timer(std::chrono::milliseconds(time), std::bind(&ViconNode::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            driver.run();
        }
        mocap::ViconDriver driver;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("vicon node"), "Vicon Node Started ");
    auto vicon_node = std::make_shared<ViconNode>();
    
    rclcpp::spin(vicon_node);
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("vicon node"),"Vicon Node Shutting Down");

    return 0;
}
