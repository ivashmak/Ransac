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

        std::vector<int> random_numbers;
        for (int i = 0; i < npoints; i++) {
            int rand_number;
            // Generate a random number that has not already been used.
            while (std::find(random_numbers.begin(),
                             random_numbers.end(),
                             (rand_number = distribution(generator))) !=
                   random_numbers.end()) {
            }

            random_numbers.push_back(rand_number);
            sample[i] = rand_number;
        }
    }

};


#endif //RANSAC_UNIFORMSAMPLER_H
