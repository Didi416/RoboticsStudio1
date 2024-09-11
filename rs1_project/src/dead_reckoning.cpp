#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DeadReckoning : public rclcpp::Node {
    public:
        DeadReckoning() : Node("dead_reckoning"){
            this->declare_parameter<bool>("_linear", false); //set default to false
            this->get_parameter("_linear", linearVel_); //read parameter from command line
            this->declare_parameter<bool>("_angular", false); //set default to false
            this->get_parameter("_angular", angularVel_); //read parameter from command line
            this->declare_parameter<bool>("_distance", false); //set default to false
            this->get_parameter("_distance", distance_); //read parameter from command line
            this->declare_parameter<bool>("_direction", false); //set default to false
            this->get_parameter("_direction", direction_); //read parameter from command line

            cmdPub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        };

        void moveRobot(){
            geometry_msgs::msg::Vector3 linear;
            geometry_msgs::msg::Vector3 angular;
            geometry_msgs::msg::Twist msgPub;

            //do stuff here to calculate linear and angular velocities

            msgPub.linear = linear;
            msgPub.angular = angular;

            cmdPub->publish(msgPub);
        }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub;
    double linearVel_;
    double angularVel_;
    double distance_;
    double direction_;
};