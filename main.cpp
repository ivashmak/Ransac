#include <stdio.h>
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "Generator/generator.h"
#include "Detector/detector.h"
#include "Ransac/Line2DEstimator.h"
#include "Ransac/Ransac.h"

int main () {

	auto total_begin = std::chrono::steady_clock::now();
	
	srand (time(NULL));
	
	std::vector<cv::Point_<float>> points;
	generate(points);
	std::cout << "generated image\n";
	
	// std::vector<cv::KeyPoint> points = detect("data/image1.jpg", "sift");
	// std::cout << "detected points\nstart clock\n";
	
	Model model(10, 2, 0.99, "ransac");
	Sampler sampler(points);
	TerminationCriteria termination_criteria (model);
	Quality quality;

	Line2DEstimator estimator(points);
	Ransac naive_ransac (points, model, sampler, termination_criteria, quality, estimator);
	std::vector<cv::Point_<float>> line(2);
	
	naive_ransac.run(points, line);
	

	auto total_end = std::chrono::steady_clock::now();
	
	std::cout << "Naive Ransac time: " << naive_ransac.quality->getComputationTime() << "ms\n";
	std::cout << "Naive Ransac iterations: " << naive_ransac.quality->getIterations() << "\n";
	std::cout << "Naive Ransac points under threshold: " << naive_ransac.quality->getNumberOfPointsUnderThreshold() << "\n";
	
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";

	naive_ransac.quality->showResult(model, points, line);
	return 0;
}
