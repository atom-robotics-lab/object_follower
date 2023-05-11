#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
class ImageProcessing{
    public:
    void process_image(cv::Mat image){
        cv::Mat blurred;
        cv::Mat hsv;
        cv::Mat mask;
        cv::Mat erod;
        cv::Mat dilt;
        cv::Mat conts;
        cv::GaussianBlur(image, blurred, cv::Size(11,11),0);
        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(29 ,86, 6), cv::Scalar(64, 255, 255), mask);
        cv::erode(mask, erod, 2);
        cv::dilate(erod, dilt, 2);

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(conts, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::namedWindow("MASK IMAGE", cv::WINDOW_NORMAL);
        cv::imshow("MASK IMAGE", mask);
        cv::waitKey(0);

    }
};

int main(int argc, char** argv) {    

    ros::init(argc, argv, "image_display_node");
    ros::NodeHandle nh;
    std::string filename = "/home/noemoji041/ros_workspaces/gcc_catkin/src/object_follower/object_follower_sim/src/object.png";
    cv::Mat image = cv::imread(filename);
    ImageProcessing ok;
    ok.process_image(image);

    return 0;
}
