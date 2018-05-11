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

// using namespace cv;
// using namespace std;
// using namespace cv::xfeatures2d;
// using namespace std::chrono;

int main () {
	auto total_begin = std::chrono::steady_clock::now();
	
	srand (time(NULL));
	
	std::vector<cv::Point2f> keypoints = generate();
	std::cout << "generated image\n";
	
	// std::vector<cv::KeyPoint> keypoints = detect("data/image1.jpg", "sift");
	// std::cout << "detected keypoints\nstart clock\n";
	cv::InputArray points (keypoints);

	Model model(10, 2, 0.99, "ransac");
	Sampler sampler(points);
	TerminationCriteria termination_criteria (model);
	Quality quality;

	NaiveRansac naive_ransac(points, model, sampler, termination_criteria, quality);
	
	Line best_line = naive_ransac.getBestLineFit();

	auto total_end = std::chrono::steady_clock::now();
	
	std::cout << "Naive Ransac time: " << naive_ransac.quality->getComputationTime() << "ms\n";
	std::cout << "Naive Ransac iterations: " << naive_ransac.quality->getIterations() << "\n";
	std::cout << "Naive Ransac points under threshold: " << naive_ransac.quality->getNumberOfPointsUnderThreshold() << "\n";
	
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";

	naive_ransac.quality->showResult(model, points, best_line);
	return 0;
}
