#ifndef RANSAC_UNIFORMSAMPLER_H
#define RANSAC_UNIFORMSAMPLER_H

#include "../Estimator/Estimator.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"

// https://stackoverflow.com/questions/288739/generate-random-numbers-uniformly-over-an-entire-range

class UniformSampler : public Sampler {
private:
    ArrayRandomGenerator * randomGenerator;
public:
    UniformSampler (bool reset_time=true) {
        randomGenerator = new ArrayRandomGenerator;
        if (reset_time) {
            randomGenerator->resetTime();
        }
    }

    void initRandomGenerator () override {
        randomGenerator->resetGenerator(0, points_size-1);
        randomGenerator->setSubsetSize(sample_size);
    }

    void generateSample (int *sample) override {
        randomGenerator->generateUniqueRandomSet(sample);
    }

    bool isInit () override {
        return sample_size != 0 && points_size != 0 && randomGenerator->isInit();
    }
};


#endif //RANSAC_UNIFORMSAMPLER_H
