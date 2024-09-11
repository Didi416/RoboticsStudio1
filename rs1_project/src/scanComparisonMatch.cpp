#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

class Localiser : public rclcpp::Node {
public:
    Localiser() : Node("scan_to_image_node"){
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Localiser::scanCallback, this, std::placeholders::_1));
        // cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&Localiser::mapCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Localiser Node started.");
        // cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        occupancyGridToImage(msg);

        mapImage_ = m_MapColImage.clone();

        cv::rotate(mapImage_, mapImage_, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow("Map Image", mapImage_);
        cv::waitKey(1);
        map_image_captured_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)

        scanImage_ = laserScanToMat(msg);
        // cv::imshow("Scan Image", scanImage_);
        // cv::waitKey(1);
        
        if (map_image_captured_) {
            cv::Point2f initialPose(0,0);
            localiseTurtleBot(mapImage_, scanImage_, initialPose, map_scale_);  
        }
    }

    int localiseTurtleBot(const cv::Mat& mapSection, const cv::Mat& laserScan, const cv::Point2f& initialPose, float mapResolution) {
               
        // Detect and match features between the map and laser scan
        std::vector<cv::Point2f> mapPoints, scanPoints;
        detectAndMatchFeatures(mapSection, laserScan, mapPoints, scanPoints);
        std::cout<<mapPoints.size()<<" and "<<scanPoints.size()<<std::endl;

        if (mapPoints.empty() || scanPoints.empty()) {
            // If no good matches were found, return the initial pose
            std::cerr << "No good feature matches found!" << std::endl;
            return 0;
        }
        // Estimate transformation between scan points and map points using findHomography
        cv::Mat homography = cv::findHomography(scanPoints, mapPoints, cv::RANSAC);

        if (homography.empty()) {
            // If the homography cannot be computed, return the initial pose
            std::cerr << "Homography computation failed!" << std::endl;
            return 0;
        }

        // Apply the transformation (homography) to estimate the robot's new pose
        std::vector<cv::Point2f> untransformedPoints(1, initialPose); // Use robot origin as reference
        std::vector<cv::Point2f> transformedPoints;
        cv::perspectiveTransform(untransformedPoints, transformedPoints, homography);

        // Convert the transformed point to pixel coordinates
        // cv::Point2f targetPose = transformedPoints[0] * mapResolution;
        if (untransformedPoints == transformedPoints){
            std::cout << "Localised successfully!" << std::endl;
            rotateRobot();
        }

        // geometry_msgs::msg::Twist movementCmd = calculateMovement(initialPose, targetPose);

        // // Publish the movement commands to the TurtleBot
        // cmd_publisher_->publish(movementCmd);

        // // Add a small delay to let the robot execute the movement
        // rclcpp::sleep_for(std::chrono::seconds(2));
        return 0;
    }

    void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 1.0;  // Rotate with some angular velocity
        cmd_publisher_->publish(twist_msg);

        // Sleep for a while to allow the robot to rotate
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    geometry_msgs::msg::Twist calculateMovement(const cv::Point2f& currentPose, const cv::Point2f& targetPose) {

        geometry_msgs::msg::Twist cmdVel;

        // Compute the error between current and target pose
        float errorX = targetPose.x - currentPose.x;
        float errorY = targetPose.y - currentPose.y;

        // Convert the error into a magnitude and angle
        float distance = sqrt(errorX * errorX + errorY * errorY);
        float angleToTarget = atan2(errorY, errorX);

        // Adjust the velocity based on the distance and angle
        // Simple proportional control for movement and rotation
        cmdVel.linear.x = std::min(0.2f, 0.1f * distance); // Cap forward speed at 0.2 m/s
        cmdVel.angular.z = 0.2f * angleToTarget; // Proportional rotation control

        return cmdVel;
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) { //convert laser scan to cv image format
        // Parameters
        float max_range = scan->range_max;
        //create blank image of 500x500 pixels
        cv::Mat image = cv::Mat::zeros(imageSize_x, imageSize_y, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            //iterate through laser scan
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                unsigned int x = static_cast<int>((range * cos(angle)) * imageSize_x / (2 * max_range)) + imageSize_x / 2;
                unsigned int y = static_cast<int>((range * sin(angle)) * imageSize_y / (2 * max_range)) + imageSize_y / 2;
                if (x >= 0 && x < imageSize_x && y >= 0 && y < imageSize_y) {
                    image.at<uchar>(y, x) = 255; //if within range, mark with a white dot on image
                }
            }
        }
        return image;
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (15% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid){
        grid_ = *grid;
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        imageSize_x = grid->info.width;
        imageSize_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                                   0, 1, 0,
                                                   0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        std::cout << "Occupancy grid map converted to a binary image\n";

        // Display the image to verify
        cv::imshow("Occupancy Grid", m_MapColImage);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    cv::Mat mapImage_, scanImage_;
    bool map_image_captured_ = false;
    bool second_image_captured_ = false;

    nav_msgs::msg::OccupancyGrid grid_;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int imageSize_x;
    unsigned int imageSize_y;

    const std::string WINDOW1 = "Map Image";
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localiser>());
    rclcpp::shutdown();
    return 0;
}