#ifndef USAC_FUNDAMENTALESTIMATOR_H
#define USAC_FUNDAMENTALESTIMATOR_H

#include "Estimator.h"
#include "Fundamental/FundemantalSolver.h"

class FundamentalEstimator : public Estimator {
private:
    const float * const points;
    cv::Mat F;
    float *F_ptr;
public:

    /*
     * input_points must be:
     * img1_x1 img1_y1 img2_x1 img2_y1
     * img1_x2 img1_y2 img2_x2 img2_y2
     * ....
     * img1_xN img1_yN img2_xN img2_yN
     */

    /*
     * x^T F x = 0
     */
    FundamentalEstimator(cv::InputArray input_points) : points((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    void setModelParameters (const cv::Mat& model) override {
        F = cv::Mat_<float>(model);

        /*
         * To make pointer from Mat class, this Mat class should exists as long as exists pointer
         * So this->F and this->F_inv must be global in class
         */
        F_ptr = (float *) F.data;
    }

    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        cv::Mat_<float> F; // use global

        unsigned int roots = SevenPointsAlgorithm(points, sample, F);

//        std::cout << "Roots " << roots << "\n\n";

        for (unsigned int i = 0; i < roots;) {
            if (!all_ori_valid(F.rowRange(i * 3, i * 3 + 3), sample, 7)) {
//                std::cout << "BAD FUNDAMENTAL MATRIX ORIENTATION. continue\n";
                roots--;
                continue;
            }
            models[i]->setDescriptor(F.rowRange(i * 3, i * 3 + 3));
            i++;
        }

        return roots;
    }

    bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, Model &model) override {
        cv::Mat_<float> F;

        if (! EightPointsAlgorithm(points, sample, sample_size, F)) {
            return false;
        }

        model.setDescriptor(F);

        return true;
    }

    /*
     * Sampson error
     *                               (pt2^t * F * pt1)^2)
     * Error =  -------------------------------------------------------------------
     *          (((F⋅pt1)(0))^2 + ((F⋅pt1)(1))^2 + ((F^t⋅pt2)(0))^2 + ((F^t⋅pt2)(1))^2)
     *
     * [ x2 y2 1 ] * [ F(1,1)  F(1,2)  F(1,3) ]   [ x1 ]
     *               [ F(2,1)  F(2,2)  F(2,3) ] * [ y1 ]
     *               [ F(3,1)  F(3,2)  F(3,3) ]   [ 1  ]
     *
     */
    float GetError(unsigned int pidx) override {
        unsigned int smpl = 4*pidx;
        float x1 = points[smpl];
        float y1 = points[smpl+1];
        float x2 = points[smpl+2];
        float y2 = points[smpl+3];


        float F_pt1_x = F_ptr[0] * x1 + F_ptr[1] * y1 + F_ptr[2];
        float F_pt1_y = F_ptr[3] * x1 + F_ptr[4] * y1 + F_ptr[5];

        // Here F is transposed
        float F_pt2_x = F_ptr[0] * x2 + F_ptr[3] * y2 + F_ptr[6];
        float F_pt2_y = F_ptr[1] * x2 + F_ptr[4] * y2 + F_ptr[7];

        float pt2_F_pt1 = x2 * F_pt1_x + y2 * F_pt1_y + F_ptr[6] * x1 +  F_ptr[7] * y1 +  F_ptr[8];

        float error = (pt2_F_pt1 * pt2_F_pt1) / (F_pt1_x * F_pt1_x + F_pt1_y * F_pt1_y + F_pt2_x * F_pt2_x + F_pt2_y * F_pt2_y);

        // debug
//        cv::Mat pt1 = (cv::Mat_<double>(3,1) << x1, y1, 1);
//        cv::Mat pt2 = (cv::Mat_<double>(3,1) << x2, y2, 1);
//        cv::Mat F_double = cv::Mat_<double> (F);
//        double error_opencv = cv::sampsonDistance(pt1, pt2, F_double);
//        if (fabsf (error - (float) error_opencv) > 2) {
//            std::cout << "error " << error << " VS opencv error " << error_opencv << "\n";
//            std::cout << "difference " << fabsf (error - (float) error_opencv) << "\n";
//            std::cout << "Check GetError in Fundamental Matrix Estimator!\n";
//        }
        //

        // std::cout << "error = " << error << '\n';
        // error >= 0
        return error;
    }

    void getModelbyCameraMatrix (const cv::Mat &K1, const cv::Mat &K2, const cv::Mat &E, cv::Mat &F) override {
        F =  K2.inv().t() * E * K1.inv();
    }

    int SampleNumber() override {
        return 7;
    }


    void GetFundamentalFromProjectionMats(const cv::Mat &P1, const cv::Mat &P2, cv::Mat &F) {
        cv::Mat e1;
        cv::SVD svd(P1, 4);

        e1 = svd.vt.row(3);
        e1 = e1.t();

        cv::Mat e2 = P2 * e1;

        cv::Mat e2x = (cv::Mat_<float>(3, 3) << 0, -e2.at<float>(2), e2.at<float>(1),
                                                e2.at<float>(2), 0, -e2.at<float>(0),
                                               -e2.at<float>(1), e2.at<float>(0), 0);

        F = e2x * P2 * P1.inv(cv::DECOMP_SVD);
        F = F / F.at<float>(2, 2);
    }



private:
    // https://github.com/danini/graph-cut-ransac/blob/master/GraphCutRANSAC/essential_estimator.cpp
    /************** oriented constraints ******************/
    void epipole(cv::Mat &ec, const cv::Mat &F) const {
        ec = F.row(0).cross(F.row(2));

        for (int i = 0; i < 3; i++)
            if ((ec.at<float>(i) > 1.9984e-15) || (ec.at<float>(i) < -1.9984e-15)) return;
        ec = F.row(1).cross(F.row(2));
    }

    // u = x1 y1 1 x2 y2 1
    float getorisig(const cv::Mat &F, const cv::Mat *ec, unsigned int pt_idx) const {
        float s1, s2;
        cv::Mat pt;

//        float x1 = points[pt_idx];
        float y1 = points[pt_idx+1];
        float x2 = points[pt_idx+2];
        float y2 = points[pt_idx+3];

//        s1 = F->at<float>(0) * u.at<float>(3) + F->at<float>(3) * u.at<float>(4) + F->at<float>(6) * u.at<float>(5);
//        s2 = ec->at<float>(1) * u.at<float>(2) - ec->at<float>(2) * u.at<float>(1);

        s1 = F.at<float>(0) * x2 + F.at<float>(3) * y2 + F.at<float>(6);
        s2 = ec->at<float>(1) - ec->at<float>(2) * y1;

        return(s1 * s2);
    }

    bool all_ori_valid(const cv::Mat &F, const int * const sample, int N) const {
        cv::Mat ec;
        float sig, sig1;
        int i;
        epipole(ec, F);

//        sig1 = getorisig(F, &ec, data.row(sample[0]));
        sig1 = getorisig(F, &ec, 4*sample[0]);

        for (i = 1; i < N; i++) {
//            sig = getorisig(F, &ec, data.row(sample[i]));
            sig = getorisig(F, &ec, 4*sample[i]);

            if (sig1 * sig < 0) return false;
        }
        return true;
    }
};

#endif //USAC_FUNDAMENTALESTIMATOR_H