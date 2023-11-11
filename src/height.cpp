#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat frame, hsv_image, mask;
int minHue = 0, maxHue = 179;
int minSaturation = 0, maxSaturation = 255;
int minValue = 0, maxValue = 255;

void onTrackbar(int value, void* userdata)
{
    cv::Scalar lower_bound(minHue, minSaturation, minValue);
    cv::Scalar upper_bound(maxHue, maxSaturation, maxValue);

    cv::inRange(hsv_image, lower_bound, upper_bound, mask);
    cv::imshow("Mask", mask);
}

void findheight()
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::Mat approximated_contour;
        cv::approxPolyDP(contour, approximated_contour, epsilon, true);

        if (approximated_contour.rows == 4) {
            double height = cv::norm(approximated_contour.at<cv::Point2f>(0) - approximated_contour.at<cv::Point2f>(2));
            std::cout << "Stick height: " << height << std::endl;
            cv::polylines(frame, approximated_contour, true, cv::Scalar(0, 255, 0), 2);
        }
    }
    cv::imshow("Camera Feed with Polygons", frame);
}

void wfovImageCallback(const wfov_camera_msgs::WFOVImageConstPtr& msg)
{
    // Convert the WFOVImage to a sensor_msgs::Image
    sensor_msgs::Image image_raw=msg->image;
    // image.header = msg->header;
    // image. = msg->height;
    // image.width = msg->width;
    // image.encoding = msg->encoding;
    // image.is_bigendian = msg->is_bigendian;
    // image.step = msg->step;
    // image.data = msg->data;

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image;

        cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);
        onTrackbar(0, 0);
        findheight();

        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Min Hue", "Mask", &minHue, 180, onTrackbar);
    cv::createTrackbar("Max Hue", "Mask", &maxHue, 180, onTrackbar);
    cv::createTrackbar("Min Saturation", "Mask", &minSaturation, 255, onTrackbar);
    cv::createTrackbar("Max Saturation", "Mask", &maxSaturation, 255, onTrackbar);
    cv::createTrackbar("Min Value", "Mask", &minValue, 255, onTrackbar);
    cv::createTrackbar("Max Value", "Mask", &maxValue, 255, onTrackbar);

    // Subscribe to the WFOVImage topic and convert it to sensor_msgs::Image
    ros::Subscriber wfov_image_sub = nh.subscribe("/camera/image", 1, wfovImageCallback);

    ros::spin();
    cv::destroyAllWindows();

    return 0;
}
