import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

def main():
    # Set ROS Master URI and Hostname inside the script
    os.environ["ROS_MASTER_URI"] = "http://192.168.50.53:11311"  # Replace with Raspberry Pi IP
    os.environ["ROS_HOSTNAME"] = "192.168.50.53"  # Replace with Raspberry Pi IP for mobile robot

    rospy.init_node("image_publisher_node", anonymous=True)
    image_publisher = rospy.Publisher("/robot_camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    # Initialize the camera
    camera = cv2.VideoCapture(0)  # Adjust camera ID if necessary
    if not camera.isOpened():
        rospy.logerr("Failed to open camera")
        return

    # Time interval for capturing images
    capture_interval = 45  # Default: 45 seconds

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_publisher.publish(ros_image)
            rospy.loginfo("Published an image")
        else:
            rospy.logerr("Failed to capture image")

        time.sleep(capture_interval)

    camera.release()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
