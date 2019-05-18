
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "lane.cpp"


// IMAGE BLURRING
int main() {
    LaneDetector laneDetector;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    double turn;
    int flag_plot = 0;
    cv::Mat frame = cv::imread("/home/mustang/Pictures/New/cam.jpg", CV_LOAD_IMAGE_COLOR);
    if(frame.empty())
    {
        std::cout<<"image not loaded";
    }
    else {
        std::cout << "Width : " << frame.cols << std::endl;
        std::cout << "Height: " << frame.rows << std::endl;
        cv::resize(frame, frame, cv::Size(320, 240));

        img_denoise = laneDetector.deNoise(frame);

        img_edges = laneDetector.edgeDetector(img_denoise);

        img_mask = laneDetector.mask(img_edges);

        lines = laneDetector.houghLines(img_mask);

        std::cout<<lines.data()<<std::endl;
        if (!lines.empty()) {
            // Separate lines into left and right lines
            left_right_lines = laneDetector.lineSeparation(lines, img_edges);

            // Apply regression to obtain only one line for each side of the lane
            lane = laneDetector.regression(left_right_lines, frame);

            turn = laneDetector.predictTurn();
            std::cout << static_cast<double>(turn) << std::endl;
            // Plot lane detection
            flag_plot = laneDetector.plotLane(frame, lane, turn);
            //std::cout << flag_plot;

            cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
            cv::imshow("frame", frame);
            cv::imshow("test", img_denoise);
            cv::imshow("edges", img_edges);
            cv::imshow("mask", img_mask);
            cv::waitKey(0);
        }
        else
            std::cout<<"HoughLines returned empty vector"<<std::endl;


    }

    return 0;
}