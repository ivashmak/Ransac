#include <stdio.h>
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "Generator/generator.h"
#include "Detector/detector.h"
#include "Ransac/NaiveRansac.h"
#include "Ransac/AbstractRansac.h"

// using namespace cv;
// using namespace std;
// using namespace cv::xfeatures2d;
// using namespace std::chrono;

int main () {
	auto total_begin = std::chrono::steady_clock::now();
	
	srand (time(NULL));
	
	generate();
	std::cout << "generated image\n";

	std::vector<cv::KeyPoint> keypoints = detect("data/image1.jpg", "sift");
	
	std::cout << "detected keypoints\nstart clock\n";
	
	Ransac *ransac;
	NaiveRansac naive_ransac;

	// ransac = &naive_ransac;

	// Line best_line;// = ransac(keypoints);

	auto total_end = std::chrono::steady_clock::now();
	
	std::cout << "Ransac time: " << " " << "ms\n";
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";

	return 0;
}
