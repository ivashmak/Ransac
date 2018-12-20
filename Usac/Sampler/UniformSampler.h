#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
private:
    unsigned int * points_random_pool;
    unsigned int current_point_idx;

    void shuffleArray () {
        unsigned int max = points_size;
        for (int i = 0; i < points_size; i++) {
            unsigned int array_random_index = (unsigned int) random () % max;
            unsigned int temp = points_random_pool[array_random_index];
            max--;
            points_random_pool[array_random_index] = points_random_pool[max];
            points_random_pool[max] = temp;
        }
        current_point_idx = 0;
    }

public:
    UniformSampler (bool reset_time=true) {
        if (reset_time) {
            srand (time(NULL));
        }
    }

    void setSampleSize (unsigned int sample_size_) override {
        sample_size = sample_size_;
    }

    void setPointsSize (unsigned int points_size_) override {
        if (points_size < points_size_) {
            points_size = points_size_;
            points_random_pool = new unsigned int[points_size];
            for (unsigned int i = 0; i < points_size; i++) {
                points_random_pool[i] = i;
            }
            shuffleArray();
        } else {
            points_size = points_size_;
        }

    }


    void generateSample (int * sample) override {
        /*
         * Shuffle if required sample size is bigger than points size.
         * I we shuffle inside sample loop, we can get repeated sample.
         */
        if (current_point_idx + sample_size >= points_size) {
            shuffleArray();
            current_point_idx = 0;
        }

        for (unsigned int i = 0; i < sample_size; i++) {
            sample[i] = points_random_pool[current_point_idx++];
//            std::cout << sample[i] << " ";
        }
//        std::cout << '\n';
    }

    bool isInit () override {
        return sample_size != 0 && points_size != 0;
    }
};


#endif //RANSAC_UNIFORMSAMPLER_H
