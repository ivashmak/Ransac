#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"

class Line2DEstimator : public Estimator {
    public:
        void EstimateModel(cv::InputArray input_points, int *sample, Model &model) {
            const int idx1 = sample[0];
            const int idx2 = sample[1];
            cv::Point_<float> *points = (cv::Point_<float> *) input_points.getMat().data;

            // Estimate the model parameters from the sample
            float tangent_x = points[idx2].x - points[idx1].x;
            float tangent_y = points[idx2].y - points[idx1].y;

            float a = tangent_y;
            float b = -tangent_x;
            float mag = static_cast<float>(sqrt(a * a + b * b));
            a /= mag;
            b /= mag;
            float c = (points[idx2].x * points[idx1].y - points[idx2].y * points[idx1].x)/mag;

            // Set the model descriptor
            cv::Mat descriptor;
            descriptor = (cv::Mat_<float>(1,3) << a, b, c);
            model.setDescriptor(descriptor);
        }

        void EstimateModelNonMinimalSample(cv::InputArray input_points, int *sample, Model &model) {}

        float GetError(cv::InputArray input_points, int pidx, Model * model) {
            cv::Point_<float> * points = (cv::Point_<float> *) input_points.getMat().data;
            cv::Mat descriptor;

            model->getDescriptor(descriptor);
            auto * params = reinterpret_cast<float *>(descriptor.data);

            return abs(params[0] * points[pidx].x + params[1]*points[pidx].y + params[2]);
        }
        
        int SampleNumber() {
        	return 2;
        } 
};


#endif //RANSAC_LINE2DESTIMATOR_H