#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;

class RobotRunTimeManager : public rclcpp::Node
{
    public:
        RobotRunTimeManager():Node("robot_runtime_manager")
        {
            mStatePublisher = this->create_publisher<std_msgs::msg::String>("/robot_state",10);
            mTimer = this->create_wall_timer(200ms,std::bind(&RobotRunTimeManager::update,this));
            mCmdVelSuber = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,
            std::bind(&RobotRunTimeManager::onCmdVel,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"robot_runtime_manager started");
        }


    private:

        enum class RuntimeState
        {
            Idle,
            Manual,
            Auto,
            Paused,
            Fault,
            Emergency
        };

        std::string stateToString(RuntimeState aState) const
        {
            switch (aState)
            {
                case RuntimeState::Idle:
                    return "Idle";
                case RuntimeState::Manual:
                    return "Manual"; 
                case RuntimeState::Auto:
                    return "Auto";
                case RuntimeState::Paused:
                    return "Paused";
                case RuntimeState::Fault:
                    return "Fault";
                case RuntimeState::Emergency:
                    return "Emergency";
            }
            return "UnKnown";
        }

        void publishState()
        {
            std_msgs::msg::String msg;
            msg.data = stateToString(mState);
            mStatePublisher->publish(msg);
            RCLCPP_INFO(this->get_logger(),"current state is  %s",msg.data.c_str());
        }
        void update()
        {
            mState = decideState();
            publishState();
        }

        void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr aMsg)
        {
            mHasCmdVel = true;
            mLastCmdVelTime = this->now();

            mLastLinearX = aMsg->linear.x;
            mLastAngularZ = aMsg->angular.z;
        }

        RuntimeState decideState()
        {
            auto nowTime = this->now();
            auto cmdTimeout = rclcpp::Duration::from_seconds(1.0);

            bool cmdActive = mHasCmdVel && nowTime - mLastCmdVelTime < cmdTimeout && (std::fabs(mLastLinearX) > 0.001 ||std::fabs(mLastAngularZ) > 0.001);
            
            if(cmdActive)
                return RuntimeState::Manual;
            return RuntimeState::Idle;
        }

        bool mHasCmdVel{false};
        rclcpp::Time mLastCmdVelTime;
        double mLastLinearX{0.0};
        double mLastAngularZ{0.0};

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mStatePublisher;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mCmdVelSuber;
        rclcpp::TimerBase::SharedPtr mTimer;
        RuntimeState mState{RuntimeState::Idle};
};


int main(int argc ,char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotRunTimeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}