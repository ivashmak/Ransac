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
//                points.at<float>(i,0) = points_arr[sample[i]].x;
//                points.at<float>(i,1) = points_arr[sample[i]].y;
                points.at<float>(i,0) = points_arr[i].x;
                points.at<float>(i,1) = points_arr[i].y;
            }
            std::cout << "pts = " << points << ";\n";

            cv::Scalar mean1 = cv::mean(points.col(0));
            cv::Scalar mean2 = cv::mean(points.col(1));

            cv::Mat means = (cv::Mat_<float> (1,2) << mean1.val[0], mean2.val[0]);
            cv::Mat ones = cv::Mat::ones(model.sample_number, 1, CV_32FC1);


            cv::Mat centered = points - ones*means;

            cv::Mat covar1, covar2, mu1, mu2;

            cv::calcCovarMatrix(points, covar1, mu1, CV_COVAR_NORMAL | CV_COVAR_ROWS);
            covar1 = covar1 / (points.rows - 1);

            cv::calcCovarMatrix(centered, covar2, mu2, CV_COVAR_NORMAL | CV_COVAR_ROWS);
            covar2 = covar2 / (centered.rows - 1);

            cv::Mat centeredT;
            cv::transpose(centered, centeredT);
            cv::Mat C = centeredT*centered;
            C = C / (model.sample_number-1);
            cv::sqrt(C, C);
            C = C / sqrt(2);
//            std::cout << "C = \n" << C << "\n\n";
//            std::cout << "cover1 = \n" << covar1 << "\n\n";
//            exit (0);

//            std::cout << covar1 << "\n\n";
//            std::cout << covar2 << "\n\n";
//            std::cout << means << "\n\n";
//            std::cout << mu1 << "\n\n";
//            std::cout << mu2 << "\n\n";



            cv::Mat eigenvals_mat, eigenvecs;
            std::vector<float> eigenvals(2);

            cv::eigen (C, eigenvals, eigenvecs);
//            cv::eigen (C, eigenvals_mat, eigenvecs);

//            std::cout << "eigenvals = \n" << eigenvals_mat << "\n\n";
//            std::cout << eigenvals[0] << " " << eigenvals[1] << "\n\n";

            float a, b, c;
            a = -eigenvals[0];
            b = -eigenvals[1];
            c = (float) (-a*mean1.val[0] - b*mean2.val[0]);

            std::cout << a << "x + " << b << "y + " << c << " = 0\n";
            std::cout << "y = " << (-a/b) << "*x + " << c << ";\n";


            cv::Mat image = cv::imread("../data/image1.jpg");
            int width = image.cols;
            int height = image.rows;

            for (int i = 0; i < model.sample_number; i++) {
                circle(image, points_arr[sample[i]], 3, cv::Scalar(255, 0, 0), -1);
            }
            Drawing draw;
            draw.draw_function(-a/b, -c, std::max(width, height), cv::Scalar(0,0,255), image);
            imshow("estimate", image);
            cv::waitKey (0);


            //            exit (0);

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