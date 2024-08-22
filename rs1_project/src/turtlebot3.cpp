#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>

class TurtleBotLaserProcessing : public rclcpp::Node {
  public:
  TurtleBotLaserProcessing() : Node("laserProcessing"){
    scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&TurtleBotLaserProcessing::topicCallback, this, std::placeholders::_1));
    scanPublisher1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan/subset_1", 10);
    scanPublisher2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan/subset_2", 10);
    filteredScanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan/filteredScan", 10);
    count = 0;
  }

  void topicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // float givenValue = 2;
    // for distance at givenValue (Display the range reading for a given angle of scan):
    // RCLCPP_INFO_STREAM(this->get_logger(), "Ranges: " << msg->ranges.at(givenValue));
    // Select a subset of range values from the scan (-pi/4,pi/4) and republish only the selected subset of range data using a different topic
    // double angle_min_deg_1 = 0.0;
    // double angle_max_deg_1 = 30.0;
    // double angle_min_deg_2 = 330.0;
    // double angle_max_deg_2 = 360.0;
    // double angle_min_rad_1 = angle_min_deg_1 * M_PI / 180.0;
    // double angle_max_rad_1 = angle_max_deg_1 * M_PI / 180.0;
    // double angle_min_rad_2 = angle_min_deg_2 * M_PI / 180.0;
    // double angle_max_rad_2 = angle_max_deg_2 * M_PI / 180.0;
    // int start_index_1 = static_cast<int>((angle_min_rad_1 - msg->angle_min) / msg->angle_increment);
    // int end_index_1 = static_cast<int>((angle_max_rad_1 - msg->angle_min) / msg->angle_increment);
    // int start_index_2 = static_cast<int>((angle_min_rad_2 - msg->angle_min) / msg->angle_increment);
    // int end_index_2 = static_cast<int>((angle_max_rad_2 - msg->angle_min) / msg->angle_increment);
    // auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    // subset_scan_1->ranges = std::vector<float>(msg->ranges.begin() + start_index_1, msg->ranges.begin() + end_index_1 + 1);
    // subset_scan_1->angle_min = msg->angle_min + start_index_1 * msg->angle_increment;
    // subset_scan_1->angle_max = msg->angle_min + end_index_1 * msg->angle_increment;
    // auto subset_scan_2 = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    // subset_scan_2->ranges = std::vector<float>(msg->ranges.begin() + start_index_2, msg->ranges.begin() + end_index_2 + 1);
    // subset_scan_2->angle_min = msg->angle_min + start_index_2 * msg->angle_increment;
    // subset_scan_2->angle_max = msg->angle_min + end_index_2 * msg->angle_increment;
    // scanPublisher1_->publish(*subset_scan_1);
    // scanPublisher2_->publish(*subset_scan_2);
    
    // display every nth scan range
    if(count == 0){
      std::cout<<"nthNum: ";
      std::cin>>nthNum;
      count = 1;
    }
    
    auto filteredScan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    filteredScan->header = msg->header;
    filteredScan->angle_min = msg->angle_min;
    filteredScan->angle_max = msg->angle_max;
    filteredScan->angle_increment = msg->angle_increment * nthNum;
    filteredScan->time_increment = msg->time_increment;
    filteredScan->scan_time = msg->scan_time;
    filteredScan->range_min = msg->range_min;
    filteredScan->range_max = msg->range_max;

    filteredScan->ranges.resize(msg->ranges.size()/nthNum);
    for (long unsigned int i = 0; i < msg->ranges.size(); i += nthNum) {
      filteredScan->ranges.at(i / nthNum) = msg->ranges.at(i);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Current angle: " << filteredScan->angle_increment);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "OG angle: " << msg->angle_increment << " and new: " << msg->angle_increment*nthNum);
    filteredScanPub_->publish(*filteredScan);
  }

  private:
  sensor_msgs::msg::LaserScan::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPublisher1_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPublisher2_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filteredScanPub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber_;
  float nthNum;
  int count;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<TurtleBotLaserProcessing>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}