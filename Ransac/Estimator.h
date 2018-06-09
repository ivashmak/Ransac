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
    virtual void EstimateModel(cv::InputArray points, cv::OutputArray &line, int *sample, int sample_nubmer, Model &model) = 0;
    virtual void EstimateModelNonMinimalSample(cv::InputArray points, int *sample, int sample_nubmer, Model &model) = 0; 
    virtual void GetError(Model &model, cv::InputArray points) = 0;
    virtual int SampleNumber()  = 0; 

/*	void exitOnFail(void) {
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
    }*/
    
};

#endif //RANSAC_ESTIMATOR_H