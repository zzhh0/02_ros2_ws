#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "my_interface/msg/robot_pose.hpp"

class PoseMonitor : public rclcpp::Node
{
    public:
    PoseMonitor():Node("pose_monitor")
    {
        mSubscription = this->create_subscription<geometry_msgs::msg::Pose2D>("/robot_pose",10,
        std::bind(&PoseMonitor::poseCallBack,this,std::placeholders::_1));

        mMyPoseSub = this->create_subscription<my_interface::msg::RobotPose>("/my_robot_pose",10,std::bind(
            &PoseMonitor::myPoseCallBack,this,std::placeholders::_1
        ));
    }
    private:
        void poseCallBack(const geometry_msgs::msg::Pose2D::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(),"monitor pose: x=%.2f y=%.2f theta=%.2f",msg->x,msg->y,msg->theta);
        }

        void myPoseCallBack(const my_interface::msg::RobotPose::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(),"monitor my pose x=%.2f  y=%.2f theta=Z%.2f linear=%.2f angular=%.2f state=%s",
            msg->x,msg->y,msg->theta,msg->linear_speed,msg->angular_speed,msg->state.c_str());
        }
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr mSubscription;
        rclcpp::Subscription<my_interface::msg::RobotPose>::SharedPtr mMyPoseSub;
};


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PoseMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
