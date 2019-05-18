#ifndef SMART_CAR_LANE_H
#define SMART_CAR_LANE_H
/*#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
*/
class LaneDetector {
private:
    double img_size;
    double img_center;
    bool left_flag = false;
    bool right_flag = false;
    cv::Point right_b;
    double right_m;  // y = m*x + b
    cv::Point left_b;
    double left_m;

public:
    cv::Mat deNoise(cv::Mat inputImage);
    cv::Mat edgeDetector(cv::Mat img_noise);
    cv::Mat mask(cv::Mat img_edges);
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);
    double predictTurn();
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, double turn);
};


#endif //TEST_CAR_LANE_H
