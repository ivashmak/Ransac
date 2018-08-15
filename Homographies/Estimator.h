#ifndef HOMOGRAPHIES_ESTIMATOR_H
#define HOMOGRAPHIES_ESTIMATOR_H

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

class Estimator {
public:
    virtual void DLT(cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) = 0;
    virtual void NormalizedDLT(cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) = 0;
    virtual void GetNormalizingTransformation (cv::InputArray pts, cv::Mat &T, cv::Mat &offset, float * s, float *s1, float * s2)= 0;
};

#endif //HOMOGRAPHIES_ESTIMATOR_H