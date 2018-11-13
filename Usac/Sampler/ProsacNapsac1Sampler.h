#ifndef USAC_PROSACNAPSAC1SAMPLER_H
#define USAC_PROSACNAPSAC1SAMPLER_H

#include "Sampler.h"

class ProsacNapsac1Sampler : public Sampler {
private:
    RandomGenerator * random_generator;
    int knn;
public:
    /*
     * Select the first point by PROSAC. Then the rest by NAPSAC.
     */
    bool isInit () override {
        return true;
    }
};


#endif //USAC_PROSACNAPSAC1SAMPLER_H
