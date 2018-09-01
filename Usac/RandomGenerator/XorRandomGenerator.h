#ifndef USAC_XORRANDOMGENERATOR_H
#define USAC_XORDRANDOMGENERATOR_H

#include "RandomGenerator.h"

class XorRandomGenerator : public RandomGenerator {
protected:
    unsigned int g_seed = 0;
    unsigned int N_points;
public:
    XorRandomGenerator (unsigned int N_points) {
        this->N_points = N_points;
    }

    int getRandomNumber () override {
        static uint32_t x = time(0);
        static uint32_t y = time(0)/2;
        static uint32_t z = time(0)/3;
        static uint32_t w = time(0)/5;
        uint32_t t;
        t = x ^ (x << 11);
        x = y; y = z; z = w;
        w = (w ^ (w >> 19) ^ (t ^ (t >> 8))) % N_points;
        return w;

    }

    void generateUniqueRandomSample (int * sample) override {

    }

    void resetGenerator (int min_range, int max_range) override {

    }

};

#endif //USAC_XORRANDOMGENERATOR_H
