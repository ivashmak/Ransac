#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H

#include "Estimator.h"
#include "DLT/DLT.h"

class HomographyEstimator : public Estimator{
private:
    const float * const points;
    cv::Mat H, H_inv;
    float *H_ptr, *H_inv_ptr;
public:

    /*
     * input_points must be:
     * img1_x1 img1_y1 img2_x1 img2_y1
     * img1_x2 img1_y2 img2_x2 img2_y2
     * ....
     * img1_xN img1_yN img2_xN img2_yN
     *
     */

    HomographyEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (const cv::Mat& model) override {
        H = cv::Mat_<float>(model); // clone
//        H = model.clone();
        H_inv = H.inv();

        /*
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->H and this->H_inv must be global in class
         */
        H_ptr = (float *) H.data;
        H_inv_ptr = (float *) H_inv.data;
    }

    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        cv::Mat H;
        DLT (points, sample, 4, H);
        // normalize H by h33
        H = H / H.at<float>(2,2);

        models[0]->setDescriptor(H);

        return 1;
    }

    bool EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat H;
        if (! NormalizedDLT(points, sample, sample_size, H)) {
            std::cout << "Normalized DLT failed\n";
            return false;
        }
        // normalize H by last h33
        H = H / H.at<float>(2,2);

        model.setDescriptor(H);
        return true;
    }

    bool LeastSquaresFitting (const int * const sample, int sample_size, Model &model) override {
        return EstimateModelNonMinimalSample(sample, sample_size, model);

        EstimateModelNonMinimalSample(sample, sample_size, model);
//        std::cout << "H pca " << model.returnDescriptor() << "\n\n";

        cv::Mat H;
        if (! NormalizedDLTLeastSquares(points, sample, sample_size, H)) {
            return false;
        }
        // normalize H by last h33
        H = H / H.at<float>(2,2);

//        std::cout << "H lsq " << H << "\n\n";

        model.setDescriptor(H);
        return true;
    }
    /*
     * Error = distance (pt(i)H, pt'(i)) + distance (pt(i), pt'(i)H^-1)
     */
    float GetError(int pidx) override {
        float error = 0;
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];

        float est_x2 = H_ptr[0] * x1 + H_ptr[1] * y1 + H_ptr[2];
        float est_y2 = H_ptr[3] * x1 + H_ptr[4] * y1 + H_ptr[5];
        float est_z2 = H_ptr[6] * x1 + H_ptr[7] * y1 + H_ptr[8];

        est_x2 /= est_z2;
        est_y2 /= est_z2;

        float est_x1 = H_inv_ptr[0] * x2 + H_inv_ptr[1] * y2 + H_inv_ptr[2];
        float est_y1 = H_inv_ptr[3] * x2 + H_inv_ptr[4] * y2 + H_inv_ptr[5];
        float est_z1 = H_inv_ptr[6] * x2 + H_inv_ptr[7] * y2 + H_inv_ptr[8];

        est_x1 /= est_z1;
        est_y1 /= est_z1;

        error += sqrt ((x2 - est_x2) * (x2 - est_x2) + (y2 - est_y2) * (y2 - est_y2));
        error += sqrt ((x1 - est_x1) * (x1 - est_x1) + (y1 - est_y1) * (y1 - est_y1));

        return error/2;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H