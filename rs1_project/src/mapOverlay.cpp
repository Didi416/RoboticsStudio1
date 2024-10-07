#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int offset_x = 0;
int offset_y = 0;
int img1_og_width = 0;
int img1_og_height = 0;
int scale_percent = 100;
cv::Mat img1, img2, blendedImg;
double alpha;

// Callback function for trackbars, not needed but required by OpenCV
void on_trackbar(int, void*){}

void on_scale_change(int, void*) {
 
    // This function will be called whenever the scale trackbar is adjusted
    double scale = scale_percent / 100.0;

    // Resize images based on the current scale
    cv::resize(img1, img1, cv::Size(img1_og_width*scale, img1_og_height*scale));

}

int main(int argc, char** argv) {
    // Step 1: Load the two map images
    if (argc != 4){
        std::cerr<<"Usage: "<<argv[0]<<" <path_to_map1> <path_to_map2> <alpha>" <<std::endl;
        return -2;
    }

    std::string map1_path = argv[1];
    std::string map2_path = argv[2];
    alpha = std::stod(argv[3]);

    img1 = cv::imread(map1_path, cv::IMREAD_GRAYSCALE);
    img2 = cv::imread(map2_path, cv::IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()){
        std::cerr << "Error: Could not load the images!" << std::endl;
        return -1;
    }

    // Resize the image
    int original_width = img1.cols;
    int original_height = img1.rows;
    double target_width = 600;
    float aspect_ratio = static_cast<float>(original_height) / original_width;
    int new_height = static_cast<int>(target_width * aspect_ratio);
    cv::resize(img1, img1, cv::Size(target_width, new_height));
    img1_og_width = img1.cols;
    img1_og_height = img1.rows;

    original_width = img2.cols;
    original_height = img2.rows;
    target_width = 600;
    aspect_ratio = static_cast<float>(original_height) / original_width;
    new_height = static_cast<int>(target_width * aspect_ratio);
    cv::resize(img2, img2, cv::Size(target_width, new_height));

    cv::imshow("Generated Map", img1);
    cv::imshow("SLAM Map", img2);

    // alpha = std::stod(argv[3]);  // Adjust the opacity (0.0 = fully transparent, 1.0 = fully opaque)
    // cv::Mat overlay;
    // cv::addWeighted(img1, 1.0, img2, alpha, 0.0, img2);

    int window_width = std::max(img1.cols, img2.cols) * 1.5;
    int window_height = std::max(img1.rows, img2.rows) * 1.5;
    cv::Mat canvas(window_height, window_width, CV_8UC1, cv::Scalar(255)); // Initialize with white background

    const char* window_name = "Maps Overlay";
    cv::namedWindow(window_name);

    // Step 3: Create trackbars to control the x and y offsets
    cv::createTrackbar("X Offset", window_name, &offset_x, window_width, on_trackbar);
    cv::createTrackbar("Y Offset", window_name, &offset_y, window_height, on_trackbar);
    cv::createTrackbar("Scale %", window_name, &scale_percent, 150, on_scale_change); // Scale from 50% to 200%

    while (true) {
        // Create a blended image
        blendedImg = cv::Mat::zeros(cv::max(img1.rows, img2.rows) + offset_y,
                            cv::max(img1.cols, img2.cols) + offset_x,
                            img1.type());

        // Place the first image in the center
        cv::Mat roi1 = blendedImg(cv::Rect(0, 0, img1.cols, img1.rows));
        img1.copyTo(roi1);

        // Adjust opacity of the second image
        cv::Mat roi2 = blendedImg(cv::Rect(offset_x, offset_y, img2.cols, img2.rows));
        cv::addWeighted(roi2, 1 - alpha, img2, alpha, 0.0, roi2);

        // Display the blendedImg image
        cv::imshow("Overlay Image", blendedImg);
        // Break loop if the user presses 'q'
        char key = (char)cv::waitKey(1);
        if (key == 'q') {
            break;
        }
    }
    //     // Step 4: Reset the canvas for each iteration
        // canvas = cv::Mat(window_height, window_width, CV_8UC1, cv::Scalar(255)); // Reset canvas to white

    //     // double scale_factor = scale_percent / 100.0;
    //     // cv::Mat scaled_img1;
    //     // cv::resize(img1, scaled_img1, cv::Size(), scale_factor, scale_factor);

    //     // Handle boundary conditions (to avoid out of bounds)
    //     // if (x1 >= 0 && y1 >= 0 && x1 + img1.cols <= canvas.cols && y1 + img1.rows <= canvas.rows) {
    //     //     img1.copyTo(canvas(roi1));
    //     // }
    //     // if (x2 >= 0 && y2 >= 0 && x2 + img2.cols <= canvas.cols && y2 + img2.rows <= canvas.rows) {
    //     //     img2.copyTo(canvas(roi2));
    //     // }

    //     // Step 8: Display the result
    // cv::imshow(window_name, canvas);

    //     
    // }

    // // Step 2: Detect keypoints and extract descriptors using ORB
    // cv::Ptr<cv::ORB> orb = cv::ORB::create();
    // std::vector<cv::KeyPoint> keypoints1, keypoints2;
    // cv::Mat descriptors1, descriptors2;
    // orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);
    // orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);
    // // Step 3: Match keypoints using BFMatcher
    // cv::BFMatcher matcher(cv::NORM_HAMMING);
    // std::vector<cv::DMatch> matches;
    // matcher.match(descriptors1, descriptors2, matches);
    // // Step 3: Sort matches based on their distance
    // std::sort(matches.begin(), matches.end());
    // // Step 4: Filter good matches using distance threshold
    // const float good_match_percent = 0.35f;  // Keep the top 25% of matches
    // int num_good_matches = static_cast<int>(matches.size() * good_match_percent);
    // matches.erase(matches.begin() + num_good_matches, matches.end());
    // // Step 5: Draw the good matches
    // cv::Mat img_matches;
    // cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    // // Step 6: Display the matched points
    // cv::imshow("Matches", img_matches);

    // // Filter out good matches using a distance threshold
    // double max_dist = 0;
    // for (const auto &m : matches) {
    //     if (m.distance > max_dist) max_dist = m.distance;
    // }
    // std::vector<cv::DMatch> good_matches;
    // for (const auto &m : matches) {
    //     if (m.distance < 0.3 * max_dist) {  // Adjust this threshold as needed
    //         good_matches.push_back(m);
    //     }
    // }
    // // Step 4: Find homography to align the two images
    // std::vector<cv::Point2f> points1, points2;
    // for (const auto &m : good_matches) {
    //     points1.push_back(keypoints1[m.queryIdx].pt);
    //     points2.push_back(keypoints2[m.trainIdx].pt);
    // }
    // cv::Mat H = cv::findHomography(points2, points1, cv::RANSAC);
    // // Warp img2 to align with img1
    // cv::Mat img2_aligned;
    // cv::warpPerspective(img2, img2_aligned, H, img1.size());

    // Step 5: Reduce opacity of img2
    ;

    // Step 6: Display the overlaid images
    // cv::imshow("Overlayed Maps", overlay);
    cv::waitKey(0);

    // Save the result if needed
    // cv::imwrite("overlayed_map.pgm", overlay);

    return 0;
}
