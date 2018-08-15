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
	std::cout << "generated points\n";
	
	// std::vector<cv::KeyPoint> points = detect("data/image1.jpg", "sift");
	// std::cout << "detected points\nstart clock\n";

	Model model(10, 2, 0.99, "ransac");
	UniformSampler sampler;
	TerminationCriteria termination_criteria (model);
	Quality quality;

	Estimator *estimator2d = new Line2DEstimator;

	Ransac naive_ransac (points, model, sampler, termination_criteria, quality);

	naive_ransac.run(points, estimator2d);


	auto total_end = std::chrono::steady_clock::now();

    std::cout << "Naive Ransac time: " << naive_ransac.quality->getComputationTime() << "mcs\n";
	std::cout << "Naive Ransac iterations: " << naive_ransac.quality->getIterations() << "\n";
	std::cout << "Naive Ransac points under threshold: " << naive_ransac.quality->getNumberOfPointsUnderThreshold() << "\n";
	
	std::cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>
													(total_end - total_begin).count() << "ms\n";


	Drawing drawing;
    cv::Mat image = cv::imread("../data/image1.jpg");

    drawing.showInliers(points, naive_ransac.most_inliers, image);

    drawing.draw_model(naive_ransac.best_model, std::max (image.cols, image.rows), cv::Scalar(0, 0, 255), image);
    drawing.draw_model(naive_ransac.non_minimal_model, std::max (image.cols, image.rows), cv::Scalar(255, 0, 0), image);


    imshow("Inliers", image);
    cv::waitKey (0);

//	if (naive_ransac.best_sample.size() != 0) {
//        drawing.showResult(model, points, naive_ransac.best_sample);
//    } else {
//        drawing.showInliers(points, naive_ransac.most_inliers);
//    }

	return 0;
}
