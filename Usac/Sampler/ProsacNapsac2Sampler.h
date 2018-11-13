#ifndef USAC_PROSACNAPSAC2SAMPLER_H
#define USAC_PROSACNAPSAC2SAMPLER_H

#include "Sampler.h"

class ProsacNapsac2Sampler : public Sampler {
private:
    RandomGenerator * random_generator;
    int knn;
public:
    /*
     * Select the first point by PROSAC. Then from the neighborhood select still by PROSAC.
     */
    bool isInit () override {
        return true;
    }
};


#endif //USAC_PROSACNAPSAC2SAMPLER_H
