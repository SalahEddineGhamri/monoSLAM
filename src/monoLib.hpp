// This a header for mono library
// 
#ifndef MONOLIB_HPP
#define MONOLIB_HPP

//dependecies
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
//--------------------------------------------
// Global variables
//--------------------------------------------

std::vector<cv::Point2f> DetectFeatures( cv::Mat &frame );
void FastFeatureDetection ( std::vector<cv::Point2f> &features, cv::Mat &frame );
void FeatureTracker (cv::Mat &prvFrame, cv::Mat &frame, std::vector<cv::Point2f> &prvFeatures, std::vector<cv::Point2f> &features);
void DrawFeatures (cv::Mat &frame, std::vector<cv::Point2f> &corners, cv::Scalar color);
void PoseRecover (std::vector<cv::Point2f> &prvFeatures, std::vector<cv::Point2f> &features, cv::Mat &R, cv::Mat &t);

#endif