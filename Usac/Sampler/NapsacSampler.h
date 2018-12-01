#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "Sampler.h"
#include "../Helper/Drawing/Drawing.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"

/*
 * https://pdfs.semanticscholar.org/cec1/2adbb307124e0c62efbaaa870836c3846b5f.pdf
 * http://www.bmva.org/bmvc/2002/papers/164/full_164.pdf
 *
 */
class NapsacSampler : public Sampler {
private:
    RandomGenerator * random_generator;
    int knn;
    const int * const neighbors;
    int * next_neighbors;
public:

    ~NapsacSampler() {
        delete next_neighbors;
    }

    /*
     * @k_nearest_neighbors_indices is ordered matrix by distance of neighbors
     * Size is points_size (N) x k nearest neighbors (K)
     * neighbor1_of_(x1,y1) ... neighborK_of_(x1,y1)
     * ...
     * neighbor1_of_(xN,yN) ... neighborK_of_(xN,yN)
     */
    NapsacSampler (const int * const k_nearest_neighbors_indices, int knn_, unsigned int sample_size_, unsigned int points_size_, bool reset_time = true)
            : neighbors (k_nearest_neighbors_indices) {

//        std::cout << k_nearest_neighbors_indices.getMat() << "\n";

        assert (k_nearest_neighbors_indices != nullptr);

        this->knn = knn_;
        this->sample_size = sample_size_;
        this->points_size = points_size_;
        
        random_generator = new ArrayRandomGenerator;
        random_generator->resetGenerator(0, points_size-1);

        if (reset_time) random_generator->resetTime();

        // allocate as zeros (starting from 1 neighbors)
        next_neighbors = (int *) calloc (points_size, sizeof (int));
    }

    /*
     * Take uniformly one initial point.
     * Take (sample_size-1) points from initial point neighborhood.
     */
    void generateSample (int *sample) override {

        int initial_point = random_generator->getRandomNumber();
        sample[0] = initial_point;

//        std::cout << sample[0] << " ";
        for (int i = 1; i < sample_size; i++) {
            // Get the farthest neighbor
            sample[i] = neighbors[(knn * initial_point) + next_neighbors[initial_point] + knn-1];
//            // move next neighbor
//            next_neighbors[initial_point] = (next_neighbors[initial_point] - 1) % -knn;
            next_neighbors[initial_point]--;
            if (next_neighbors[initial_point] == -knn) {
                next_neighbors[initial_point] = 0;
            }
//            std::cout << sample[i] << " ";
        }
//        std::cout << '\n';
    }

    bool isInit () override {
        return true;
    }
};

#endif //USAC_NAPSACSAMPLER_H
