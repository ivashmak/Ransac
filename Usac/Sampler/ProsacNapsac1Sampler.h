#ifndef USAC_PROSACNAPSAC1SAMPLER_H
#define USAC_PROSACNAPSAC1SAMPLER_H

#include "Sampler.h"
#include "NapsacSampler.h"
#include "ProsacSampler.h"

class ProsacNapsac1Sampler : public Sampler {
private:
    RandomGenerator * random_generator;
    int knn;
public:
    /*
     * Select the first point by PROSAC. Then the rest by NAPSAC.
     */
    void initSampler (unsigned int knn, unsigned int sample_size, unsigned int points_size, const int * const neighbors) {
        Sampler * prosac = new ProsacSampler;
        Sampler * napsac = new NapsacSampler (neighbors, knn, sample_size, points_size);
    }

    void generateSampler (int * sample) override {

    }

    bool isInit () override {
        return true;
    }
};


#endif //USAC_PROSACNAPSAC1SAMPLER_H
