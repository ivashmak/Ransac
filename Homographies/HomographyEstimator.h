#ifndef HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H
#define HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H


#include "Estimator.h"

class HomographyEstimator : public Estimator{
    void DLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H);
    void NormalizedDLT(cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H);
    void GetNormalizingTransformation (cv::InputArray pts, cv::Mat &T, cv::Mat &offset, float * s, float *s1, float * s2);

};


#endif //HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H
