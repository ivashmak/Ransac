#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"


bool fit_point_between_lines (float x, float y, float k, float b1, float b2);

class Line2DEstimator : public Estimator {
    public:
        cv::Point_<float> *points;

        Line2DEstimator (cv::InputArray points) {
            CV_Assert(!points.empty());

            this->points = (cv::Point_<float> *) points.getMat().data;
        }

        void EstimateModel(cv::InputArray points, cv::OutputArray &line, int *sample, int sample_number, Model &model) {}
        void EstimateModelNonMinimalSample(cv::InputArray points, int *sample, int sample_number, Model &model) {}
        
        void GetError(Model &model, cv::InputArray points) {}
        
        float GetError2(cv::InputArray r_points, int kp) {
            cv::Point_<float> *random_points = (cv::Point_<float> *) r_points.getMat().data;
            cv::Point_<float> p1 = random_points[0];
            cv::Point_<float> p2 = random_points[1];
            return abs((p2.y-p1.y)*points[kp].x - (p2.x-p1.x)*points[kp].y + p2.x*p1.y - p2.y*p1.x)/sqrt(pow(p2.y-p1.y,2)+pow(p2.x-p1.x,2));       
        }
        
        int SampleNumber() {
        	return 2;
        } 
};


#endif //RANSAC_LINE2DESTIMATOR_H