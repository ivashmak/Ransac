#ifndef USAC_PROSACNAPSAC1SAMPLER_H
#define USAC_PROSACNAPSAC1SAMPLER_H

#include "Sampler.h"
#include "NapsacSampler.h"
#include "ProsacSampler.h"

class ProsacNapsacSampler : public Sampler {
private:
    RandomGenerator * random_generator;
    int knn;
public:
    /*
        1. Select a point by PROSAC
        2. Select the rest of the required points from the selected points's neighborhood.
            Not randomly but, again, with PROSAC using the score of the neighbours.
        3. If the neighborhood is not big enough (there are not enough points for estimating a model),
            select a new one by PROSAC (step (1)), and continue.
    */
    void initSampler (unsigned int knn, unsigned int sample_size, unsigned int points_size, const int * const neighbors) {
        Sampler * prosac = new ProsacSampler;
    }

    void generateSample (int * sample) override {

    }

    bool isInit () override {
        return true;
    }
};


#endif //USAC_PROSACNAPSAC1SAMPLER_H
