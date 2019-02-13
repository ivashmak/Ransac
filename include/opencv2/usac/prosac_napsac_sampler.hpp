// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_PROSACNAPSACSAMPLER_H
#define USAC_PROSACNAPSACSAMPLER_H

#include "sampler.hpp"
#include "napsac_sampler.hpp"
#include "prosac_sampler.hpp"
#include "prosac_simple_sampler.hpp"

class ProsacNapsacSampler : public Sampler {
private:
    ProsacSimpleSampler * prosac_simple_sampler;

    // represents map between neighborhood of point and prosac sampling
    // key is index of point
    std::map<int, ProsacSimpleSampler> prosac_neighborhood;
    int knn;
    int initial_point[1];

    int * neighbors;
    std::vector<std::vector<int>> neighbors_v;
    NeighborsSearch search = NeighborsSearch ::NullN;

public:
    /*
        1. Select a point by PROSAC
        2. Select the rest of the required points from the selected points's neighborhood.
            Not randomly but, again, with PROSAC using the score of the neighbours.
        3. If the neighborhood is not big enough (there are not enough points for estimating a model),
            select a new one by PROSAC (step (1)), and continue.
    */
    ProsacNapsacSampler (const Model * const model, unsigned int points_size_, const int * const neighbors) {
        prosac_simple_sampler = new ProsacSimpleSampler (1 /*sample size */, points_size_, model->reset_random_generator);
        knn = model->k_nearest_neighbors;
        sample_size = model->sample_size;
    }

    /*
     * Assume all points are sorted by quality function in decreasing order from
     * the best to the worst.
     * And all neighbors for every point are sorted in the same way too.
     */
    void setNeighbors (cv::InputArray neighbors_, NeighborsSearch search_) {
        search = search_;
        assert(! neighbors_.empty());
        assert(search != NeighborsSearch::NullN);

        if (search == NeighborsSearch::Nanoflann) {
            assert(knn >= sample_size-1);
            neighbors = (int *) neighbors_.getMat().data;
        } else {
            neighbors_v = *(std::vector<std::vector<int>>*) neighbors_.getObj();
        }
    }

    /*
     * Generate points from 2D vector structure.
     */
    void generateSampleVector (int * sample) {
        unsigned int attempts = 0;

        int init_pt;
        // Try to find initial points by prosac where enough points in its neighborhood.
        while (attempts < 1000) {
            prosac_simple_sampler->generateSample(initial_point);
            if (neighbors_v[initial_point[0]].size() < sample_size-1) {
                attempts++;
                continue;
            }
            break;
        }
        init_pt = initial_point[0];

        if (attempts == 1000) {
            std::cout << "There are very few neighbors. Can not sample from neighborhood!\n";
            exit (111);
        }

        // add prosac sampling to current initial point, if key is not there yet.
        if (prosac_neighborhood.find(init_pt) == prosac_neighborhood.end()) {
            // sampling from points in neighborhood of initial point
            prosac_neighborhood[init_pt] = ProsacSimpleSampler(sample_size, neighbors_v[init_pt].size(), true);
        }

        /* get sample from neighborhood
         * sample range is from 0 to points is neighborhood-1
         */
        prosac_neighborhood[init_pt].generateSample(sample);
        // rearrange sample from neighborhood.
        for (int i = 1; i < sample_size; i++) {
            sample[i] = neighbors_v[init_pt][sample[i]];
        }
        // initial points is the first sample.
        sample[0] = init_pt;
    }

    /*
     * Generate sample from 1D array of size points_size * knn
     */
    void generateSampleArray (int * sample) {
        prosac_simple_sampler->generateSample(initial_point);
        int init_pt = initial_point[0];


        // add prosac sampling to current initial point, if key is not there yet
        if (prosac_neighborhood.find(init_pt) == prosac_neighborhood.end()) {
            // sampling from of knn points.
            prosac_neighborhood[init_pt] = ProsacSimpleSampler(sample_size, knn, true);
        }

        /*
         * get sample from neighborhood
         * sample range is from <0; knn-1>
         */
        prosac_neighborhood[init_pt].generateSample(sample);
        // rearrange sample from neighborhood.
        for (int i = 1; i < sample_size; i++) {
            sample[i] = neighbors[init_pt*knn + sample[i]];
        }
        sample[0] = init_pt;
    }

    void generateSample (int * sample) override {
        if (search == NeighborsSearch::Nanoflann) {
            generateSampleArray(sample);
        } else {
            generateSampleVector(sample);
        }
    }

    bool isInit () override {
        return search != NeighborsSearch ::NullN;
    }
};


#endif //USAC_PROSACNAPSACSAMPLER_H
