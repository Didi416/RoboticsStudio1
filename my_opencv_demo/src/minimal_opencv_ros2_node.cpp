#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&MinimalImagePublisher::topicCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("circle_image_overlay", 10);
  }
 
private:
  cv_bridge::CvImagePtr msg_;

  void topicCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      msg_ = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {
      // ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

     // Draw an example circle on the video stream
    cv::circle(msg_->image, cv::Point(msg_->image.cols/2, msg_->image.rows/2), 100, CV_RGB(255,0,0));

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    // msg2_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", msg_).toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_->toImageMsg());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg2_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
