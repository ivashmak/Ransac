#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"
#include "../RandomGenerator/UniformRandomGenerator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
public:
    UniformSampler (const bool reset_time=true) {
        randomGenerator = new UniformRandomGenerator;
        if (reset_time) {
//            std::cout << "reset!\n";
            randomGenerator->resetTime();
        }
    }

    void generateSample (int *sample) override {
        randomGenerator->generateUniqueRandomSet(sample, sample_size);
    }

};


#endif //RANSAC_UNIFORMSAMPLER_H
