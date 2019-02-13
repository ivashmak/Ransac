// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_NAPSACSAMPLER_H
#define USAC_NAPSACSAMPLER_H

#include <iostream>
#include "sampler.hpp"
#include "array_random_generator.hpp"

namespace cv { namespace usac {
class NapsacSampler : public Sampler {
private:
    RandomGenerator *random_generator;
    std::vector<std::vector<int>> neighbors_v;
    NeighborsSearch search;
    int knn = 0;
    int *next_neighbors, *neighbors = 0;
    bool do_uniform = false;

public:

    ~NapsacSampler() override {
        delete[] next_neighbors;
        delete (random_generator);
    }

    /*
     * @k_nearest_neighbors_indices is ordered matrix by distance of neighbors
     * Size is points_size (N) x k nearest neighbors (K)
     * neighbor1_of_(x1,y1) ... neighborK_of_(x1,y1)
     * ...
     * neighbor1_of_(xN,yN) ... neighborK_of_(xN,yN)
     */
    NapsacSampler(const Model *const model, unsigned int points_size_, bool reset_time = true) {
        assert(points_size_ != 0);

//        std::cout << k_nearest_neighbors_indices.getMat() << "\n";
        knn = model->k_nearest_neighbors;
        sample_size = model->sample_size;
        points_size = points_size_;

        // check if nearest neighbors is enough to sample
        assert (knn >= sample_size - 1);

        random_generator = new ArrayRandomGenerator;
        if (reset_time) random_generator->resetTime();
        random_generator->resetGenerator(0, points_size - 1);
        random_generator->setSubsetSize(1);

        // allocate as zeros (starting from 1 neighbors)
        next_neighbors = (int *) calloc(points_size, sizeof(int));
    }

    void setNeighbors(cv::InputArray neighbors_, NeighborsSearch search_) {
        search = search_;
        assert(!neighbors_.empty());
        assert(search != NeighborsSearch::NullN);

        if (search == NeighborsSearch::Nanoflann) {
            neighbors = (int *) neighbors_.getMat().data;
        } else {
            neighbors_v = *(std::vector<std::vector<int>> *) neighbors_.getObj();
        }
    }

    /*
     * Take uniformly one initial point.
     * Take (sample_size-1) points from initial point neighborhood.
     */
    void generateSampleKNN(int *sample) {
        int initial_point = random_generator->getRandomNumber();
        sample[0] = initial_point;

        for (int i = 1; i < sample_size; i++) {
            // Get the farthest neighbor
            sample[i] = neighbors[(knn * initial_point) + next_neighbors[initial_point] + knn - 1];

            // move next neighbor
            next_neighbors[initial_point]--;
            if (next_neighbors[initial_point] == -knn) {
                next_neighbors[initial_point] = 0;
            }
        }
        // n1 n2 ... nk
        // first sample is nk nk-1 ... nk-m, m is sample_size
        // next sample is nk-1 nk2-2 ... nk-1-m
    }

    void generateSampleGrid(int *sample) {
        int initial_point;
        int i;
        for (i = 0; i < points_size; i++) {
            initial_point = random_generator->getRandomNumber();
            // continue if small number of neighbors
            if (neighbors_v[initial_point].size() < sample_size) continue;
            break;
        }
        if (i == points_size) {
            std::cout << "THERE IS NO ENOUGH NEIGHBORS TO SAMPLE. DO UNIFORM SAMPLING\n";
            do_uniform = true;
            return;
        }

        sample[0] = initial_point;

//        std::cout << sample[0] << " ";
        for (i = 1; i < sample_size; i++) {
            sample[i] = neighbors_v[initial_point][next_neighbors[initial_point]];

            // move next neighbor
            next_neighbors[initial_point]++;
            if (next_neighbors[initial_point] >= neighbors_v[initial_point].size()) {
                next_neighbors[initial_point] = 0;
            }
        }
//        std::cout << '\n';
    }

    void generateSample(int *sample) override {
        if (search == NeighborsSearch::Nanoflann) {
            generateSampleKNN(sample);
        } else if (!do_uniform) {
            generateSampleGrid(sample);
        } else {
            random_generator->generateUniqueRandomSet(sample);
        }
    }

    bool isInit() override {
        return knn != 0 && (neighbors != 0 || !neighbors_v.empty());
    }
};
}}

#endif //USAC_NAPSACSAMPLER_H
