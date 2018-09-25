#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
private:
    UniformRandomGenerator * randomGenerator;
public:
    UniformSampler (bool reset_time=true) {
        randomGenerator = new UniformRandomGenerator;
        if (reset_time) {
            randomGenerator->resetTime();
        }
    }

    void initRandomGenerator () override {
        randomGenerator->resetGenerator(0, points_size-1);
    }

    void generateSample (int *sample) override {
        randomGenerator->generateUniqueRandomSet(sample, sample_size);
    }

    bool isInit () override {
        return !(sample_size == 0 || points_size == 0);
    }
};


#endif //RANSAC_UNIFORMSAMPLER_H
