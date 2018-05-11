#ifndef RANSAC_ESTIMATOR_H
#define RANSAC_ESTIMATOR_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

#include "Sampler.h"
#include "Quality.h"
#include "Model.h"
#include "TerminationCriteria.h"

#include "Line.h"


class Estimator {
public:
	Model *model;
	Quality *quality;
	Sampler *sampler;
	TerminationCriteria *termination_criteria;
	
	cv::Point2f *points;
	Line best_line;
	int total_points;

	Estimator (cv::InputArray points, 
			   Model& model, 
			   Sampler& sampler, 
			   TerminationCriteria& termination_criteria, 
			   Quality& quality) {

		CV_Assert(!points.empty());
		
		this->model = &model;
		this->sampler = &sampler;
		this->termination_criteria = &termination_criteria;
		this->quality = &quality;

		this->points = (cv::Point2f *) points.getMat().data;
		this->total_points = points.size().width;	
		// CV_Assert(sampler != nullptr);
		// CV_Assert(quality != nullptr);
		// CV_Assert(termination_criteria != nullptr);
		// CV_Assert(this->model != nullptr);
		
	}

	virtual Line getBestLineFit (void) = 0;
	// virtual Line getBestLineFit (std::vector<cv::Point2f> keypoints) = 0;

	void exitOnFail(void) {
        std::cout << "Something wrong in Ransac!\nExiting\n";
        exit(0);
    }

    void exitOnFail(std::string message) {
        std::cout << message << '\n';
        exit(0);
    }

    void exitOnSuccess(void) {
        std::cout << "Ransac finished successfuly!\n";
    }

    void exitOnSuccess(std::string message) {
        std::cout << message << '\n';
    }
    
};

#endif //RANSAC_ESTIMATOR_H