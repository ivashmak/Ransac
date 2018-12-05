#include "Tests.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Estimator/HomographyEstimator.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Estimator/EssentialEstimator.h"

void Tests::initLine2D (Estimator *& estimator, const cv::Mat& points) {
    estimator = new Line2DEstimator(points);
}

void Tests::initHomography (Estimator *& estimator, const cv::Mat& points) {
    estimator = new HomographyEstimator(points);
}

void Tests::initFundamental (Estimator *& estimator, const cv::Mat& points) {
    estimator = new FundamentalEstimator(points);
}

void Tests::initEssential (Estimator *& estimator, const cv::Mat& points) {
    estimator = new EssentialEstimator(points);
}

 
void Tests::initEstimator (Estimator *& estimator, Model * model, const cv::Mat& points) {
    assert (! points.empty());
    Tests tests;
    if (model->estimator == ESTIMATOR::Line2D) {
        tests.initLine2D(estimator, points);
    } else if (model->estimator == ESTIMATOR::Homography) {
        tests.initHomography(estimator, points);
    } else if (model->estimator == ESTIMATOR::Fundamental) {
        tests.initFundamental(estimator, points);
    } else if (model->estimator == ESTIMATOR::Essential) {
        tests.initEssential(estimator, points);
    } else {
        std::cout << "UNKOWN Estimator\n";
        exit (1111);
    }
}