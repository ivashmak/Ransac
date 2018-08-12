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
#include "Ransac/UniformSampler.h"
#include "Ransac/Drawing.h"

int main () {

	auto total_begin = std::chrono::steady_clock::now();
	
	srand (time(NULL));
	
	std::vector<cv::Point_<float>> points;
	generate(points);
	std::cout << "generated image\n";
	
	// std::vector<cv::KeyPoint> points = detect("data/image1.jpg", "sift");
	// std::cout << "detected points\nstart clock\n";
	int sample_number = 3;

	Model model(10, sample_number, 0.99, "ransac");
	UniformSampler sampler;
	TerminationCriteria termination_criteria (model);
	Quality quality;

	Estimator *estimator2d = new Line2DEstimator(sample_number);

	Ransac naive_ransac (points, model, sampler, termination_criteria, quality);

	naive_ransac.run(points, estimator2d);


	auto total_end = std::chrono::steady_clock::now();

    std::cout << "Naive Ransac time: " << naive_ransac.quality->getComputationTime() << "mcs\n";
	std::cout << "Naive Ransac iterations: " << naive_ransac.quality->getIterations() << "\n";
	std::cout << "Naive Ransac points under threshold: " << naive_ransac.quality->getNumberOfPointsUnderThreshold() << "\n";
	
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";


	Drawing drawing;

	if (naive_ransac.best_sample.size() != 0) {
        drawing.showResult(model, points, naive_ransac.best_sample);
    } else {
        drawing.showInliers(points, naive_ransac.most_inliers);
    }

	return 0;
}
