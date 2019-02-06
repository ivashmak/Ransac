// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../estimator/estimator.hpp"
#include "../random_generator/array_random_generator.hpp"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
private:
    unsigned int * points_random_pool;
    int max;
public:
    ~UniformSampler() override {
        delete[] points_random_pool;
    }

    UniformSampler (bool reset_time) {
        if (reset_time) {
            srand (time(NULL));
        }
    }

    void setSampleSize (unsigned int sample_size_) override {
        sample_size = sample_size_;
    }

    void setPointsSize (unsigned int points_size_) override {
        points_size = points_size_;
        points_random_pool = new unsigned int[points_size];
        for (unsigned int i = 0; i < points_size; i++) {
            points_random_pool[i] = i;
        }
        max = points_size;
    }


    void generateSample (int * sample) override {
        for (unsigned int i = 0; i < sample_size; i++) {
            if (max == 0) {
                max = points_size;
            }
            unsigned int array_random_index = (unsigned int) random () % max;
            unsigned int random_number = points_random_pool[array_random_index];
            max--;
            points_random_pool[array_random_index] = points_random_pool[max];
            points_random_pool[max] = random_number;
            sample[i] = random_number;

//            std::cout << sample[i] << " ";
        }
//        std::cout << '\n';
    }

    bool isInit () override {
        return sample_size != 0 && points_size != 0;
    }
};


#endif //RANSAC_UNIFORMSAMPLER_H
