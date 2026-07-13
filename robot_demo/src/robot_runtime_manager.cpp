#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <algorithm>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


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

            mScanSuber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,
            std::bind(&RobotRunTimeManager::onScan,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"robot_runtime_manager started");

            this->declare_parameter<bool>("auto_mode",false);
            this->declare_parameter<bool>("paused",false);


            mLastOdomTime = this->now();
            mLastScanTime = this->now();
            mLastTfTime = this->now();
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

        void onScan(const sensor_msgs::msg::LaserScan::SharedPtr aMsg)
        {
            mHasScan = true;
            mNearestObstacle = std::numeric_limits<double>::infinity();

            for(auto range : aMsg->ranges)
            {
                if(std::isfinite(range))
                {
                    mNearestObstacle = std::min(mNearestObstacle,static_cast<double>(range));
                }
            }

            mLastScanTime = this->now();
        }

        void onOdom(const nav_msgs::msg::Odometry::SharedPtr /*aMsg*/)
        {
            mHasOdom = true;
            mLastOdomTime = this->now();
        }

        void onTf(const tf2_msgs::msg::TFMessage::SharedPtr aMsg)
        {
            if(!aMsg->transforms.empty())
            {
                mHasTf = true;
                mLastTfTime = this->now();
            }
        }

        RuntimeState decideState()
        {

            auto nowTime = this->now();

            if(mHasScan && mNearestObstacle < 0.34)
            {
                return RuntimeState::Emergency;
            }

            auto sensorTimeout = rclcpp::Duration::from_seconds(2.0);
            bool odomLost = mHasOdom && (nowTime - mLastOdomTime > sensorTimeout);
            bool scanLost = mHasScan && (nowTime - mLastScanTime > sensorTimeout);
            bool tfLost = mHasTf && (nowTime - mLastTfTime > sensorTimeout);

            if(odomLost || scanLost || tfLost)
            {
                return RuntimeState::Fault;
            }

            auto cmdTimeout = rclcpp::Duration::from_seconds(1.0);

            if(this->get_parameter("paused").as_bool())
            {
                return RuntimeState::Paused;
            }

            if(this->get_parameter("auto_mode").as_bool())
            {
                return RuntimeState::Auto;
            }

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
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mScanSuber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSuber;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr mTfSuber;
        rclcpp::TimerBase::SharedPtr mTimer;
        RuntimeState mState{RuntimeState::Idle};

        bool mHasScan{false};
        double mNearestObstacle{std::numeric_limits<double>::infinity()};
        bool mHasOdom{false};
        bool mHasTf{false};

        rclcpp::Time mLastOdomTime;
        rclcpp::Time mLastScanTime;
        rclcpp::Time mLastTfTime;
};


int main(int argc ,char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotRunTimeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
