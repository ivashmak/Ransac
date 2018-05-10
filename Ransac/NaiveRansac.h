#ifndef RANSAC_NAIVE_RANSAC_H
#define RANSAC_NAIVE_RANSAC_H

#include "AbstractRansac.h"


void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img);
bool fit_point (float x, float y, float k, float b1, float b2);

class NaiveRansac : public Ransac {
    public:
        // NaiveRansac();
        
        Line getBestLineFit (std::vector<cv::KeyPoint> keypoints);  // override;

  		void showResult (cv::Mat image);// override;
};

#endif //RANSAC_NAIVE_RANSAC_H