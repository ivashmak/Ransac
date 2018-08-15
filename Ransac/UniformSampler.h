#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "Estimator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
public:
    UniformSampler () {
        srand (time(NULL));
    }

    void getSample (int *sample, int npoints, int total_points) {
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_int_distribution<int>  distribution(0, total_points);

        for (int i = 0; i < npoints; i++) {
            sample[i] = distribution (generator);
//            sample[i] = rand() % total_points;
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
