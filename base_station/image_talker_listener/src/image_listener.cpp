#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string>

void helloCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert ROS Image to OpenCV format
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Get the current time for the filename
        std::time_t now = std::time(0);
        std::tm* local_time = std::localtime(&now);
        std::ostringstream filename;
        filename << "/home/jychpr/MR-TP-FIX/received_images/image_"
                 << (local_time->tm_year + 1900)   // YYYY
                 << std::setw(2) << std::setfill('0') << (local_time->tm_mon + 1)  // MM
                 << std::setw(2) << std::setfill('0') << local_time->tm_mday       // DD
                 << std::setw(2) << std::setfill('0') << local_time->tm_hour       // HH
                 << std::setw(2) << std::setfill('0') << local_time->tm_min        // MM
                 << std::setw(2) << std::setfill('0') << local_time->tm_sec        // SS
                 << ".jpg";

        // Save the image to disk
        cv::imwrite(filename.str(), frame);
        ROS_INFO("Saved image: %s", filename.str().c_str());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber hello_sub = nh.subscribe("/hello_topic", 10, helloCallback);
    ros::Subscriber image_sub = nh.subscribe("/image_topic", 10, imageCallback);

    // Ensure the folder exists
    system("mkdir -p /home/jychpr/MR-TP-FIX/received_images");

    ros::spin(); // Keep the node alive to process callbacks
    return 0;
}

