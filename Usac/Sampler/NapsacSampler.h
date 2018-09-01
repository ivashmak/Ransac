#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"
#include "../Helper/Drawing.h"

int factorial (int n) { return n == 0 ? 1 : n*factorial (n-1); }


class NapsacSampler : public Sampler {
protected:
    cv::Mat k_nearest_neighbors_indices;
    int knn;
    std::random_device rand_dev;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;
    cv::flann::Index * flannIndex;
    cv::Mat points;
    int k_iter = 0;
    int taking_sample_size_from_knn;
    UniformSampler * uniformSampler;
public:

    NapsacSampler (cv::InputArray points, int knn, int sample_size, int N_points, bool reset_time = true) {
        this->knn = knn;
        this->points = points.getMat();

        if (reset_time) resetTime();

        this->sample_size = sample_size;
        this->N_points = N_points;

        generator = std::mt19937(rand_dev());


        cv::flann::LinearIndexParams flannIndexParams;
        flannIndex = new cv::flann::Index (cv::Mat(N_points, 2, CV_32F, points.getMat().data).reshape(1), flannIndexParams);

        getKNearestNeighorsIndices();

        taking_sample_size_from_knn = factorial(knn)/(factorial(sample_size)*factorial(knn-sample_size));
        uniformSampler = new UniformSampler (sample_size, N_points);
    }

    void getKNearestNeighorsIndices () {
        cv::Mat dists;
        distribution = std::uniform_int_distribution<int>(0, N_points-1);
        cv::Point_<float> initial_point = points.at<cv::Point_<float>>(distribution(generator));
        cv::Mat query(1, 2, CV_32F, &initial_point);

        flannIndex->knnSearch(query, k_nearest_neighbors_indices, dists, knn);
    }



    void generateSample (int *sample) override {
        if (k_iter > taking_sample_size_from_knn) {
            getKNearestNeighorsIndices();
            k_iter = 0;
        }

        distribution = std::uniform_int_distribution<int>(0, knn-1);

        for (int i = 0; i < sample_size; i++) {
            sample[i] = k_nearest_neighbors_indices.at<int>(distribution(generator));
            for (int j = i-1; j >=0 ; j--) {
                if (sample[j] == sample[i]) {
                    i--;
                    break;
                }
            }
        }

        k_iter++;
        k_iterations++;
    }
};


#endif //USAC_NAPSACSAMPLER_H
