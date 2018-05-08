#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

#include "Generator/generator.h"
#include "Detector/detector.h"

// using namespace cv;
// using namespace std;
// using namespace cv::xfeatures2d;

int main () {
	generate();
	std::cout << "generated image\n";

	std::vector<cv::KeyPoint> keypoints = detect("data/image1.jpg", "sift");
	std::cout << "read keypoints\n";

	// for (int i = 0; i < keypoints.size(); i++) {
	// 	std::cout << keypoints[i].pt.x<<" "<<keypoints[i].pt.y << '\n';
	// }	

	return 0;
}