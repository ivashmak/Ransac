#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

#include "../RandomGenerator/RandomGenerator.h"

class Sampler {
protected:
    RandomGenerator *randomGenerator;
    int k_iterations = 0;
public:

    virtual void generateSample (int *points) = 0;

    void setSampleSize (unsigned int sample_size) {
        randomGenerator->setSampleSize(sample_size);
    }

    void setRange (int min_range, int max_range) {
        this->randomGenerator->resetGenerator(min_range, max_range);
    }

    int getNumberOfIterations () {
        return k_iterations;
    }

};

#endif //RANSAC_SAMPLER_H