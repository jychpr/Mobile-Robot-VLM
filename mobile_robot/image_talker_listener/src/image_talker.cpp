#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_talker");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher hello_pub = nh.advertise<std_msgs::String>("/hello_topic", 10);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image_topic", 10);

    // Camera setup
    cv::VideoCapture cap(0);  // Default camera
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }

    ros::Rate loop_rate(1); // Loop at 1 Hz (1 second interval)
    float countdown = 0.5; // Countdown timer for image sending

    while (ros::ok()) {
        // Publish "Hello, World!" with countdown
        std_msgs::String hello_msg;
        std::stringstream ss;

        if (countdown > 0) {
            ss << "Hello, World! In " << countdown << " seconds I will send an image.";
            hello_msg.data = ss.str();
            hello_pub.publish(hello_msg);
            ROS_INFO("Published: %s", hello_msg.data.c_str());
        } else {
            // Capture and publish image
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) {
                ROS_ERROR("Failed to capture image!");
                continue;
            }

            try {
                sensor_msgs::ImagePtr image_msg;
                image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                image_pub.publish(image_msg);
                ROS_INFO("Published an image.");
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            // Reset countdown
            countdown = 1;
        }

        // Decrement countdown
        countdown--;
        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    return 0;
}

