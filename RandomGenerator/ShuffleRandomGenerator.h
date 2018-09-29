#ifndef USAC_SHUFFLERANDOMGENERATOR_H
#define USAC_SHUFFLERANDOMGENERATOR_H

#include <vector>
#include <algorithm>
#include "RandomGenerator.h"

class ShuffleRandomGenerator : public RandomGenerator {
protected:
    unsigned int array_size = 0;
    int * array;
    unsigned int curr_idx;
    std::default_random_engine rng = std::default_random_engine {};
public:
    void resetGenerator (int min_range, int max_range) override {
        array_size = (unsigned int) max_range - min_range + 1;
        array = new int[array_size];
        curr_idx = 0;
        std::iota (array, array+array_size, 0);
        std::shuffle (array, array+array_size, rng);
    }

    int getRandomNumber () override {
        if (curr_idx == array_size) {
            curr_idx = 0;
            std::shuffle (array, array+array_size, rng);
        }

        return array[curr_idx++];
    }

    void generateUniqueRandomSet (int * sample) override {
        for (int i = 0; i < subset_size; i++) {
            sample[i] = getRandomNumber();
        }
    }

    bool isInit () override {
        return subset_size != 0 && array_size != 0;
    }

};

#endif //USAC_SHUFFLERANDOMGENERATOR_H
