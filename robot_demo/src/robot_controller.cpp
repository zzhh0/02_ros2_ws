#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <chrono>
#include <cmath>
#include "std_srvs/srv/empty.hpp"
#include <algorithm>
#include "my_interface/msg/robot_pose.hpp"
#include "my_interface/srv/reset_pose.hpp"
#include "my_interface/action/move_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>

using MoveToPose  = my_interface::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
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
        
        
            mMyResetService = this->create_service<my_interface::srv::ResetPose>("/my_reset_pose",
                std::bind(&RobotController::myResetService,this,std::placeholders::_1,std::placeholders::_2));
        
            mMoveToPoseActionServer = rclcpp_action::create_server<MoveToPose>(
                this,"/move_to_pose",
                std::bind(&RobotController::handleGoal,this,std::placeholders::_1,std::placeholders::_2),
                std::bind(&RobotController::handleCancel,this,std::placeholders::_1),
                std::bind(&RobotController::handleAccept,this,std::placeholders::_1)
            );
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

        void myResetService(const std::shared_ptr<my_interface::srv::ResetPose::Request> aRequest,std::shared_ptr<my_interface::srv::ResetPose::Response> aResponse)
        {
            //处理请求
            mX = aRequest->x;
            mY = aRequest->y;
            mTheta = aRequest->theta;

            //生成响应
            aResponse->success =  true;
            aResponse->message = "pose reset success";

            RCLCPP_INFO(this->get_logger(),"pose reset to: x=%.2f y=%.2f  theta=%.2f",mX,mY,mTheta);
        }
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MoveToPose::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(),"receive move goal: x=%.2f y=%.2f theta=%.2f",
                goal->target_x,
                goal->target_y,
                goal->target_theta
            );

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMoveToPose> goalHandle)
        {
            (void)goalHandle;

            RCLCPP_INFO(this->get_logger(),"move goal cancel requested");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        void handleAccept(const std::shared_ptr<GoalHandleMoveToPose> goalHandle)
        {
            std::thread{
                std::bind(&RobotController::executeMoveToPose,this,goalHandle)
            }.detach();
        }

        void executeMoveToPose(const std::shared_ptr<GoalHandleMoveToPose> goalHandle)
        {
            const auto goal = goalHandle->get_goal();
            auto feedback = std::make_shared<MoveToPose::Feedback>();
            auto result = std::make_shared<MoveToPose::Result>();

            rclcpp::Rate rate(10);

            while(rclcpp::ok())
            {
                const double dx = goal->target_x - mX;
                const double dy = goal->target_y - mY;
                const double distance = std::sqrt(dx*dx + dy*dy);

                if(goalHandle->is_canceling())
                {
                    mLinearSpeed = 0.0;
                    mAngularSpeed = 0.0;

                    result->success = false;
                    result->message = "move to goal canceled";

                    goalHandle->canceled(result);
                    RCLCPP_INFO(this->get_logger(),"move goal canceled");
                    return;
                }
                
                feedback->current_x = mX;
                feedback->current_y = mY;
                feedback->current_theta = mTheta;
                feedback->distance_remaining = distance;
                goalHandle->publish_feedback(feedback);

                if(distance < 0.05)
                {
                    mLinearSpeed = 0.0;
                    mAngularSpeed = 0.0;
                    mTheta = goal->target_theta;

                    result->success = true;
                    result->message = "move goal reached";

                    goalHandle->succeed(result);
                    RCLCPP_INFO(this->get_logger(),"move goal reached");
                    return;
                }

                const double targetTheta = std::atan2(dy,dx);
                double angleError = targetTheta - mTheta;

                while(angleError > M_PI)
                {
                    angleError -= 2.0 * M_PI;
                }
                while(angleError < -M_PI)
                {
                    angleError += 2.0 * M_PI;
                }

                const double maxLinear = this->get_parameter("max_linear_speed").as_double();
                const double maxAngular = this->get_parameter("max_angular_speed").as_double();

                mLinearSpeed = std::clamp(distance*0.8,0.0,maxLinear);
                mAngularSpeed = std::clamp(angleError*2.0,-maxAngular,maxAngular);

                rate.sleep();
            }
        }
        
        rclcpp::TimerBase::SharedPtr mTimer;
        geometry_msgs::msg::Twist mTwist;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mPosePublisher; //发布者
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mResetService; //服务
        rclcpp::Publisher<my_interface::msg::RobotPose>::SharedPtr mMyPosePublisher; //自定义消息发布者
        rclcpp::Service<my_interface::srv::ResetPose>::SharedPtr mMyResetService; //自定义服务
        rclcpp_action::Server<MoveToPose>::SharedPtr mMoveToPoseActionServer;  //自定义action


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
