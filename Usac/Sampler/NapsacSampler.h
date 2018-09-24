#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"
#include "../Helper/Drawing.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"
#include "../../RandomGenerator/SimpleRandomGenerator.h"

int factorial (int n) {
    int res = n;
    while (n > 2) {
        res *= --n;
    }
    return res;
}

class NapsacSampler : public Sampler {
private:
    cv::Mat k_nearest_neighbors_indices, points, dists;
    int knn, taking_sample_size_from_knn;
    cv::flann::Index * flannIndex;
    RandomGenerator * simple_randomGenerator; // random generator for sample
    RandomGenerator * array_randomGenerator; // random generator for k nearest neighbors
public:

    /*
     * find K nearest neighbors for one, compute maximum iterations by getting SampleNumber
     * from KNN using formula knn!/(sample_number!*(knn-sample_number)!
     * and reset KNN after iterations expired.
     */
    NapsacSampler (cv::InputArray input_points, int knn, unsigned int sample_size, bool reset_time = true) {
        assert (!input_points.empty());

        this->knn = knn;
        this->points = input_points.getMat();
        this->sample_size = sample_size;
        this->points_size = points.size().width;

        simple_randomGenerator = new SimpleRandomGenerator;
        array_randomGenerator = new ArrayRandomGenerator;

        simple_randomGenerator->resetGenerator(0, points_size-1);
        array_randomGenerator->resetGenerator(0, knn-1);

        if (reset_time) { array_randomGenerator->resetTime();
                          simple_randomGenerator->resetTime(); }

        cv::flann::LinearIndexParams flannIndexParams;
        flannIndex = new cv::flann::Index (cv::Mat(points_size, 2, CV_32F, input_points.getMat().data).reshape(1), flannIndexParams);

        getKNearestNeighorsIndices();

        // quantity of all possible combination of taken sample_size from k nearest neighbors
        taking_sample_size_from_knn = factorial(knn)/(factorial(sample_size)*factorial(knn-sample_size));
        k_iterations = 0;
    }

    /*
     * Get randomly initial point from all points.
     * Find k nearest neighbors using flann.
     */
    void getKNearestNeighorsIndices () {
        flannIndex->knnSearch(cv::Mat_<float>(points.at<cv::Point_<float>>(simple_randomGenerator->getRandomNumber())),
                              k_nearest_neighbors_indices, dists, knn);
    }

    /*
     * Generate sample from k nearest neighbors points.
     * If all possible sample_size combinations of k nearest neighbors were taken then
     * Generate k neighbors again
     */
    void generateSample (int *sample) override {
        if (k_iterations % taking_sample_size_from_knn) {
            getKNearestNeighorsIndices();
        }

        for (int i = 0; i < sample_size; i++) {
            sample[i] = k_nearest_neighbors_indices.at<int>(array_randomGenerator->getRandomNumber());
            for (int j = i-1; j >=0 ; j--) {
                if (sample[j] == sample[i]) {
                    i--;
                    break;
                }
            }
        }

        k_iterations++;
    }
};



#endif //USAC_NAPSACSAMPLER_H
