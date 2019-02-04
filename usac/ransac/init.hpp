// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_INIT_H
#define RANSAC_INIT_H

#include "../Estimator/Line2DEstimator.h"
#include "../Estimator/HomographyEstimator.h"
#include "../Estimator/EssentialEstimator.h"
#include "../Estimator/FundamentalEstimator.h"


void InitEstimator (Estimator *& estimator, ESTIMATOR est, const cv::Mat& points) {
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
void InitSampler (Sampler *& sampler, const Model * const model, cv::InputArray points, 
	const cv::Mat& neighbors, std::vector<std::vector<int>> ns) {

    unsigned int points_size = std::max ((int) neighbors.rows, (int) ns.size());

    if (model->sampler == SAMPLER::Uniform) {
        sampler = new UniformSampler (model->reset_random_generator);
	    sampler->setSampleSize(model->sample_size);
	    sampler->setPointsSize(points_size);

    } else if (model->sampler == SAMPLER::Prosac) {
   	    sampler = new ProsacSampler;
	    ProsacSampler * prosac_sampler = (ProsacSampler *)sampler;
	    prosac_sampler->initProsacSampler (model->sample_size, points_size);

    } else if (model->sampler == SAMPLER::Napsac) {
		sampler = new NapsacSampler(model, points_size, model->reset_random_generator);
	    if (model->neighborsType == NeighborsSearch::Nanoflann) {
	        ((NapsacSampler *) sampler)->setNeighbors(neighbors);
	    } else {
	        ((NapsacSampler *) sampler)->setNeighbors(ns);
	    }
	        
    } else if (model->sampler == SAMPLER::Evsac) {
	    sampler = new EvsacSampler(points, points_size, k_nearest_neighbors, model->sample_size);
    
    } else if (model->sampler == SAMPLER::ProgressiveNAPSAC) {
	    sampler = new ProgressiveNapsac(points, model->sample_size);
    
    } else if (model->sampler == SAMPLER::ProsacNapsac) {
    
    } else {	
        std::cout << "UNKOWN Sampler IN Init Sampler\n";
        exit (111);
    }
}

// ----------------------------------------------------------------------------------------


#endif //RANSAC_INIT_H
