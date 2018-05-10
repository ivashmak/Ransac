#include <stdio.h>
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "Generator/generator.h"
#include "Detector/detector.h"
#include "Ransac/ransac.h"

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
	
	auto ransac_time_begin = std::chrono::steady_clock::now();
	Line best_line = ransac(keypoints);
	auto ransac_time_end = std::chrono::steady_clock::now();


	// Display output
	cv::Mat img = cv::imread("data/image1.jpg");
	int width = img.cols;
	int height = img.rows;

	float k = (best_line.x2 - best_line.x1)/(best_line.y2 - best_line.y1);
	float b = (best_line.y1*best_line.x1 - best_line.y1*best_line.x2)/(best_line.y2-best_line.y1) + best_line.x1;

	img = cv::imread("data/image1.jpg");
			
	draw_function (k, b, std::max(width, height), cv::Scalar(255,0,0), img);
	imshow("Best Line",img);

	auto total_end = std::chrono::steady_clock::now();
	
	std::cout << "Ransac time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(ransac_time_end - ransac_time_begin).count() << "ms\n";
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";
		

	cv::waitKey( 0 );

	return 0;
}
