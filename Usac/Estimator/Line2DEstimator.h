#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"

class Line2DEstimator : public Estimator {
public:
        void EstimateModel(cv::InputArray input_points, int *sample, Model &model) override {
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

        void EstimateModelNonMinimalSample(cv::InputArray input_points, int *sample, int sample_size, Model &model) override {
            cv::Mat_<float> points (sample_size, 2);

            cv::Point_<float> *points_arr = (cv::Point_<float> *) input_points.getMat().data;

            for (int i = 0; i < sample_size; i++) {
                points.at<float>(i,0) = points_arr[sample[i]].x;
                points.at<float>(i,1) = points_arr[sample[i]].y;
            }

            cv::Mat covar, means, eigenvecs, eigenvals;;
            cv::calcCovarMatrix(points, covar, means, CV_COVAR_NORMAL | CV_COVAR_ROWS);
            cv::eigen (covar, eigenvals, eigenvecs);

            float a, b, c;
            a = (float) -eigenvecs.at<double>(1,0);
            b = (float) -eigenvecs.at<double>(1,1);
            c = (float) (-a*means.at<double>(0) - b*means.at<double>(1));

            cv::Mat descriptor;
            descriptor = (cv::Mat_<float>(1,3) << a, b, c);
            model.setDescriptor(descriptor);
        }


        inline float GetError(cv::InputArray input_points, int pidx, Model * const model) override {
            // ------------------  1 the fastest --------------
            cv::Point_<float> * points = (cv::Point_<float> *) input_points.getMat().data;
//            cv::Mat descriptor;
//
//            model->getDescriptor(descriptor);
//            auto * params = reinterpret_cast<float *>(descriptor.data);

            return fabsf(params[0] * points[pidx].x + params[1] * points[pidx].y + params[2]);

            // ------------------ 1.5 times slower then fisrt ----------------
//            auto * params = (float *) model->returnDescriptor().data;
//
//            return abs((int)(params[0] * input_points.getMat().at<cv::Point_<float>>(pidx).x +
//                             params[1] * input_points.getMat().at<cv::Point_<float>>(pidx).y + params[2]));

            // ------------------ 3 times slower then first ------------------
//            return abs((int)( model->returnDescriptor().at<float>(0) * input_points.getMat().at<cv::Point_<float>>(pidx).x +
//                              model->returnDescriptor().at<float>(1) * input_points.getMat().at<cv::Point_<float>>(pidx).y +
//                              model->returnDescriptor().at<float>(2)));

        }
        
        int SampleNumber() override {
        	return 2;
        }
};


#endif //RANSAC_LINE2DESTIMATOR_H