#include "monoLib.hpp"

std::vector<cv::Point2f> DetectFeatures( cv::Mat &frame) {
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDestector = false;
    double k = 0.04;
    std::vector<cv::Point2f> corners;
    cv::Mat GrayFrame;
    cv::cvtColor(frame, GrayFrame, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(GrayFrame, 
                            corners,
                            200,
                            qualityLevel,
                            minDistance,
                            cv::Mat(),
                            blockSize,
                            useHarrisDestector,
                            k);
    return corners;
}

/*
std::vector< cv::Point2f> FastFeatureDetection ( cv::Mat &frame ) {
    std::vector< cv::Point2f > corners; // will have the feature points using indexes x, y
    std::vector< cv::KeyPoint> keypoints; // keypoint contains discroptors and points
    cv::FAST(frame, keypoints, 40, true);
    cv::KeyPoint::convert(keypoints, corners, std::vector<int>()); // converts a keypoint to a point
    return corners;
}*/

void FastFeatureDetection ( std::vector<cv::Point2f> &features ,cv::Mat &frame ) {
    std::vector< cv::KeyPoint> keypoints; // keypoint contains discroptors and points
    // a feature detector using Fast algorithm, a pointer to object detector is created. 
    cv::Ptr<cv::FastFeatureDetector> featureDetector = cv::FastFeatureDetector::create();
    featureDetector->setThreshold(50);
    featureDetector->detect(frame, keypoints); // detect
    cv::KeyPoint::convert(keypoints, features, std::vector<int>()); // converts a keypoint to a point
}

void FeatureTracker ( cv::Mat &prvframe, cv::Mat &frame, std::vector<cv::Point2f> &prvFeatures, std::vector<cv::Point2f> &features) {
    cv::Size WinSize = cv::Size(21, 21); // Window size at each pyramid? level
    std::vector<float> err;
    std::vector<uchar> status;
    int Maxlevel = 3;
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, (0.01));
    cv::calcOpticalFlowPyrLK(prvframe, frame, prvFeatures, features, status,  err, WinSize, Maxlevel, criteria, 0, 0.0001);
    // get rid of features unable to be tracked
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++) {
        cv::Point2f pt = prvFeatures.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) {
             if ((pt.x<0)||(pt.y<0)) {
                 status.at(i) = 0;
                 }
            prvFeatures.erase (prvFeatures.begin() + i - indexCorrection);
            features.erase (features.begin() + i - indexCorrection);
            indexCorrection++;
     	}
    }
}

void PoseRecover(std::vector<cv::Point2f> &prvFeautres, std::vector<cv::Point2f> &features, cv::Mat &R, cv::Mat &t) {
    cv::Mat E;
    // find essential matrix
    double focal = 1;
    cv::Point2d pp = cv::Point2d(0, 0);
    E = cv::findEssentialMat(prvFeautres, features, focal, pp, cv::RANSAC, 0.998, 1.0, cv::noArray());
    // find pose <int> 
    cv::recoverPose(E, prvFeautres, features, R, t, focal, pp, cv::noArray());
}


void DrawFeatures( cv::Mat &frame, std::vector<cv::Point2f> &corners, cv::Scalar color) {
    // Function to draw features as a circles.
    int r = 2; // radius
    for( int i = 0; i < corners.size(); i++ ) {
        cv::circle( frame, 
                    corners[i],
                    r,
                    color,
                    -1, 8, 0 ); 
    }
}