#ifndef USAC_FUNDAMENTALSOLVER_H
#define USAC_FUNDAMENTALSOLVER_H

#include <opencv2/core/mat.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

class FundamentalSolver {
private:
	const float * const points;
public:
	FundamentalSolver (const float * const points_) : points (points_) {}

	bool EightPointsAlgorithm (const int * const sample, unsigned int sample_number, cv::Mat &F);
	bool EightPointsAlgorithmEigen (const int * const sample, unsigned int sample_number, cv::Mat &F);
	bool EightPointsAlgorithm (const int * const sample, const float * const weights, unsigned int sample_number, cv::Mat &F);
	unsigned int SevenPointsAlgorithm (const int * const sample, cv::Mat &F);	
};

#endif //USAC_FUNDEMANTALSOLVER_H
