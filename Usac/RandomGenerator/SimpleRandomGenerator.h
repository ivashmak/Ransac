#ifndef USAC_SIMPLERANDOMGENERATOR_H
#define USAC_SIMPLERANDOMGENERATOR_H


#include <random>
#include <vector>
#include "RandomGenerator.h"

class SimpleRandomGenerator : public RandomGenerator {
unsigned int size;

public:
    void resetGenerator (int min_range, int max_range) override {
        size = (unsigned int) max_range - min_range;

    }

    int getRandomNumber () override {
        return random () % size;
    }

    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
        for (unsigned int i = 0; i < sample_size; i++) {
            sample[i] = getRandomNumber ();
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }

};


#endif //USAC_SIMPLERANDOMGENERATOR_H
