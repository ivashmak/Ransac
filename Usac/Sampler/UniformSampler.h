#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
protected:
    std::random_device rand_dev;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;

public:
    UniformSampler () {
        srand (time(NULL));
        generator = std::mt19937(rand_dev());
    }

    void getSample (int *sample, int npoints, int total_points) {
        distribution = std::uniform_int_distribution<int>(0, total_points-1);

        for (int i = 0; i < npoints; i++) {
            sample[i] = distribution(generator);
            for (int j = i-1; j >=0 ; j--) {
                if (sample[j] == sample[i]) {
                    i--;
                    break;
                }
            }
        }
    }

};


#endif //RANSAC_UNIFORMSAMPLER_H
