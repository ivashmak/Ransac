#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"
#include "Drawing.h"

class Line2DEstimator : public Estimator {
private:
    int sample_number;
    public:
        Line2DEstimator (int sample_number) {
            this->sample_number = sample_number;
        }

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

        void EstimateModelNonMinimalSample(cv::InputArray input_points, int *sample, Model &model) {

            cv::Mat points = cv::Mat(model.sample_number, 2, CV_32FC1);

            cv::Point_<float> *points_arr = (cv::Point_<float> *) input_points.getMat().data;

            for (int i = 0; i < model.sample_number; i++) {
                points.at<float>(i,0) = points_arr[sample[i]].x;
                points.at<float>(i,1) = points_arr[sample[i]].y;
            }
//            std::cout << "pts = " << points << ";\n";

//            cv::Scalar mean1 = cv::mean(points.col(0));
//            cv::Scalar mean2 = cv::mean(points.col(1));
//            cv::Mat means1 = (cv::Mat_<float> (1,2) << mean1.val[0], mean2.val[0]);
//            cv::Mat ones = cv::Mat::ones(model.sample_number, 1, CV_32FC1);
//            cv::Mat centered = points - ones*means;

            cv::Mat covar, means;
            cv::calcCovarMatrix(points, covar, means, CV_COVAR_NORMAL | CV_COVAR_ROWS);
//            covar = covar / (points.rows - 1);

            cv::Mat eigenvecs, eigenvals;
            cv::eigen (covar, eigenvals, eigenvecs);
//            std::cout << "eigenvecs = \n" << eigenvecs << "\n\n";

            float a, b, c;
            a = (float) eigenvecs.at<double>(0,0);
            b = (float) eigenvecs.at<double>(0,1);
            c = (float) (-a*means.at<double>(0) - b*means.at<double>(1));


            cv::Mat image = cv::imread("../data/image1.jpg");
            int width = image.cols;
            int height = image.rows;

            for (int i = 0; i < model.sample_number; i++) {
                circle(image, points_arr[sample[i]], 3, cv::Scalar(255, 0, 0), -1);
            }
            
//            Drawing draw;
//            draw.draw_function(a/b, c, std::max(width, height), cv::Scalar(0,0,255), image);
//            imshow("estimate", image);
//            cv::waitKey (0);

            cv::Mat descriptor;
            descriptor = (cv::Mat_<float>(1,3) << a, b, c);
            model.setDescriptor(descriptor);
        }

        float GetError(cv::InputArray input_points, int pidx, Model * model) {
            cv::Point_<float> * points = (cv::Point_<float> *) input_points.getMat().data;
            cv::Mat descriptor;

            model->getDescriptor(descriptor);
            auto * params = reinterpret_cast<float *>(descriptor.data);

            return abs(params[0] * points[pidx].x + params[1]*points[pidx].y + params[2]);
        }
        
        int SampleNumber() {
        	return sample_number;
        } 
};


#endif //RANSAC_LINE2DESTIMATOR_H