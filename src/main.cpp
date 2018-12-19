// Main for monoSlam Project
// Author: Salah Eddine Ghamri
#include "monoLib.hpp"


// Frames ----------------------------------------------
cv::Mat frame;
cv::Mat prvFrame;
// features vectors ------------------------------------
std::vector<cv::Point2f> features;
std::vector<cv::Point2f> prvFeatures;
//------------------------------------------------------
// Rotation and translation matrices
cv::Mat R, t;

int nbr = 1;
int WIDTH = 720, HEIGHT = 608;

int main(int argc, char** argv){
  cv::VideoCapture cap("./Video/Fast.mp4");

  if ( !cap.isOpened()) {
    std::cout << "Something went wrong opening the source" << std::endl;
    return -1;
  }

  //---------------------------------------------------
  cv::namedWindow("frame", cv::WINDOW_NORMAL);
  cv::resizeWindow("frame", WIDTH, HEIGHT);
  //---------------------------------------------------

  while(true) {
    cap >> frame;
     
    // if frame is empty shutdown
    if (frame.empty()) {
      printf(" End ! ");
      break;
    }
  
    // we detect if features size is below 100
    if ( prvFeatures.size() <= 500 ) {
    // 1. Extract features from previous image using Fast algorithm.
    FastFeatureDetection(prvFeatures, frame);
    std::cout << " frame " << nbr << " > " << prvFeatures.size() << " features detected." << std::endl;
    frame.copyTo(prvFrame);
    ++nbr;
    }
    else {
    // 2. Track features extracted ( don't use a matcher use a tracker)
    // 3. Features that can't be tracked ( Out of reach ) will be eliminated.
    // features to track are in features vector
    FeatureTracker(prvFrame, frame, prvFeatures, features);
    std::cout << " features - " << features.size() << std::endl;

    // 4. Draw features on image
    DrawFeatures(frame, features, cv::Scalar(0, 250, 0));
    // --------------------------------------------
    // 5. recover pose using the essential matrix and the epipolar geometry
    // PoseRecover calculate the rotation and tranlation matrix.
    PoseRecover(prvFeatures, features, R, t);
    std::cout << " Translation: " << t << std::endl;
    std::cout << " Rotation: " << R << std::endl;
    if (prvFrame.empty()){ std::cout << " prvFrame is not ok \n"; return EXIT_FAILURE;}
    // show the image
    cv::imshow("frame", frame);
    //Transition old frame - new frame--------------------------------------
    frame.copyTo(prvFrame);
    prvFeatures = features;
    //----------------------------------------------------------------------
    ++nbr;
    if ( cv::waitKey(30) >= 0 ) break;}
  }
  return EXIT_SUCCESS;
}