#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <chrono>
#include <cmath>
#include "std_srvs/srv/empty.hpp"
#include <algorithm>
#include "my_interface/msg/robot_pose.hpp"


using namespace std::chrono_literals;

class RobotController:public rclcpp::Node
{
    public:
        RobotController():Node("robot_controller")
        {
            this->declare_parameter<double>("max_linear_speed",1.0);
            this->declare_parameter<double>("max_angular_speed",1.0);

            mSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,
            std::bind(&RobotController::subCallBack,this,std::placeholders::_1));

            mPosePublisher = this->create_publisher<geometry_msgs::msg::Pose2D>("/robot_pose",10);
            mMyPosePublisher = this->create_publisher<my_interface::msg::RobotPose>("/my_robot_pose",10);

            mTimer = this->create_wall_timer(100ms,std::bind(&RobotController::updataPose,this));

            mResetService = this->create_service<std_srvs::srv::Empty>(
                "/reset_pose",
                std::bind(&RobotController::resetService,this,std::placeholders::_1,std::placeholders::_2));
        }
    private:
        void subCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) 
        {
            const double maxLinear = this->get_parameter("max_linear_speed").as_double();
            const double maxAngular = this->get_parameter("max_angular_speed").as_double();
            mLinearSpeed = std::clamp(msg->linear.x,-maxLinear,maxLinear);
            mAngularSpeed = std::clamp(msg->angular.z,-maxAngular,maxAngular);
            RCLCPP_INFO(this->get_logger(),"receive /cmd_vel: linear= %.2f , angular = %.2f",mLinearSpeed,mAngularSpeed);
        }
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mSub;
        
        void updataPose()
        {
            constexpr double dt = 0.1;
            mX += mLinearSpeed * std::cos(mTheta)*dt;
            mY += mLinearSpeed * std::sin(mTheta)*dt;
            mTheta += mAngularSpeed * dt;
            //官方geometry pose消息
            geometry_msgs::msg::Pose2D pose;
            pose.x = mX;
            pose.y = mY;
            pose.theta =mTheta;
            mPosePublisher->publish(pose);

            //自定义pose消息
            my_interface::msg::RobotPose myPose;
            myPose.x = mX;
            myPose.y = mY;
            myPose.theta = mTheta;
            myPose.angular_speed = mAngularSpeed;
            myPose.linear_speed = mLinearSpeed;
            myPose.state = "running";
            mMyPosePublisher->publish(myPose);

            RCLCPP_INFO(this->get_logger(),"pose: x=%.2f  y=%.2f  theta=%.2f",mX,mY,mTheta);
        }


        void resetService(const std::shared_ptr<std_srvs::srv::Empty::Request> aRequest,std::shared_ptr<std_srvs::srv::Empty::Response> aResponse)
        {
            (void)aRequest;
            (void)aResponse;

            mX = 0.0;
            mY = 0.0;
            mTheta = 0.0;

            RCLCPP_INFO(this->get_logger(),"pose reset to zero ... ");
        }

        rclcpp::TimerBase::SharedPtr mTimer;
        geometry_msgs::msg::Twist mTwist;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mPosePublisher; //发布者
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mResetService;
        rclcpp::Publisher<my_interface::msg::RobotPose>::SharedPtr mMyPosePublisher;//自定义消息发布者

        double mX{0.0};
        double mY{0.0};
        double mTheta{0.0};
        double mLinearSpeed{0.0};
        double mAngularSpeed{0.0};
};


int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
