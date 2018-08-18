#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"
#include "../Helper/Drawing.h"

class NapsacSampler : public Sampler {
protected:
    cv::Mat indicies;
    int knn;
    std::random_device rand_dev;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;
public:

    NapsacSampler (cv::InputArray points, int knn) {
        this->knn = knn;

        srand (time(NULL));
        generator = std::mt19937(rand_dev());

        int total_points = points.size().width;

        cv::Mat dists, mat_points = cv::Mat(total_points, 2, CV_32F, points.getMat().data);
        cv::flann::LinearIndexParams flannIndexParams;
        cv::flann::Index flannIndex (cv::Mat(mat_points).reshape(1), flannIndexParams);

        distribution = std::uniform_int_distribution<int>(0, total_points-1);
        cv::Point_<float> initial_point = mat_points.at<cv::Point_<float>>(distribution(generator));
        cv::Mat query(1, 2, CV_32F, &initial_point);

        flannIndex.knnSearch(query, indicies, dists, knn);




        // drawing knn. Can be deleted
        std::vector<int> v_inds;
        for (int i = 0; i < knn; i++) {
            v_inds.push_back(this->indicies.at<int>(i));
        }
        Drawing draw;
        cv::Mat image = cv::imread("../images/image1.jpg");
        draw.showInliers(points, v_inds, image);
        circle(image, initial_point, 3, cv::Scalar(255, 0, 0), -1);
        imshow("Inliers", image);
        cv::waitKey (0);
        // end
    }

    void getSample (int *sample, int npoints, int total_points) {
        distribution = std::uniform_int_distribution<int>(0, knn-1);

        for (int i = 0; i < npoints; i++) {
            sample[i] = indicies.at<int>(distribution(generator));
            for (int j = i-1; j >=0 ; j--) {
                if (sample[j] == sample[i]) {
                    i--;
                    break;
                }
            }
        }
    }
};


#endif //USAC_NAPSACSAMPLER_H
