// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/init.hpp"

void cv::usac::initEstimator (Estimator *& estimator, ESTIMATOR est, const cv::Mat& points) {
    if (est == ESTIMATOR::Line2d) {
        estimator = new Line2DEstimator(points);

    } else if (est == ESTIMATOR::Homography) {
	    estimator = new HomographyEstimator(points);

    } else if (est == ESTIMATOR::Fundamental) {
	    estimator = new FundamentalEstimator(points);

    } else if (est == ESTIMATOR::Essential) {
	    estimator = new EssentialEstimator(points);

    } else {
        std::cout << "UNKOWN Estimator IN Init Estimator\n";
        exit (111);
    }
}

// ----------------------------------------------------------------------------------------
void cv::usac::initSampler (Sampler *& sampler, const Model * const model, const cv::Mat& points) {
     unsigned int points_size = points.rows;

     if (model->sampler == SAMPLER::Uniform) {
         sampler = new UniformSampler (model->reset_random_generator);
         ((UniformSampler *)sampler)->setSampleSize(model->sample_size);
         ((UniformSampler *)sampler)->setPointsSize(points_size);

     } else if (model->sampler == SAMPLER::Prosac) {
    	    sampler = new ProsacSampler;
	     ProsacSampler * prosac_sampler = (ProsacSampler *)sampler;
	     prosac_sampler->initProsacSampler (model->sample_size, points_size, model->reset_random_generator);

     } else if (model->sampler == SAMPLER::Napsac) {
		 sampler = new NapsacSampler(model, points_size);

     } else {
         std::cout << "UNKOWN Sampler IN Init Sampler\n";
         exit (111);
     }
}
void cv::usac::initTerminationCriteria (TerminationCriteria *& termination_criteria,
        const Model * const model, unsigned int points_size) {
    termination_criteria = new StandardTerminationCriteria (model, points_size);
}

void cv::usac::initProsacTerminationCriteria (TerminationCriteria *& termination_criteria, Sampler *& prosac_sampler,
            const Model * const model, Estimator * estimator, unsigned int points_size) {

    termination_criteria = new ProsacTerminationCriteria
                (((ProsacSampler *) prosac_sampler)->getGrowthFunction(), model, points_size, estimator);

    // set stopping length from prosac termination criteria to prosac sample as pointer
    ((ProsacSampler *) prosac_sampler)->setTerminationLength(((ProsacTerminationCriteria *) termination_criteria)->getStoppingLength());
    ((ProsacTerminationCriteria *) termination_criteria)->setLargestSampleSize(((ProsacSampler *) prosac_sampler)->getLargestSampleSize());
}

void cv::usac::initLocalOptimization (LocalOptimization *& local_optimization, Model * model, Estimator * estimator,
          Quality * quality, unsigned int points_size) {

     if (model->lo == LocOpt::InItLORsc || model->lo == LocOpt::InItFLORsc) {
         local_optimization = new InnerLocalOptimization (model, estimator, quality, points_size);

     } else if (model->lo == LocOpt::GC) {
        local_optimization = new GraphCut (model, estimator, quality, points_size);

     }
}

