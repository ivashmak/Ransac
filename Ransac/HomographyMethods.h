#ifndef RANSAC_HOMOGRAPHYMETHODS_H
#define RANSAC_HOMOGRAPHYMETHODS_H

#include <opencv2/core/mat.hpp>

class HomographyMethods {
public:
    void DLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H);
    void NormalizedDLT(cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H);
    void GetNormalizingTransformation (cv::InputArray pts, cv::Mat &T, cv::Mat &offset, float * s, float *s1, float * s2);
};

#endif //RANSAC_HOMOGRAPHYMETHODS_H
