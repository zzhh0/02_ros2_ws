#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
using namespace std::chrono_literals;

class CmdPublish : public rclcpp::Node
{
    public:
        CmdPublish():Node("cmd_publisher"),mCount(0)
        {
            mPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
            mTimer = this->create_wall_timer(500ms,std::bind(&CmdPublish::publish_cmd,this));
        }
    private:
        void publish_cmd()
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.5;
            msg.angular.z = 0.2;

            mPublisher->publish(msg);

            RCLCPP_INFO(this->get_logger(),"cmd_vel linear.x = %.2f  angular.z = %.2f",msg.linear.x,msg.angular.z);
        }
        rclcpp::TimerBase::SharedPtr mTimer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPublisher;
        int mCount;
};


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CmdPublish>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}