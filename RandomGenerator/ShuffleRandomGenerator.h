#ifndef USAC_SHUFFLERANDOMGENERATOR_H
#define USAC_SHUFFLERANDOMGENERATOR_H

#include <vector>
#include <algorithm>
#include "RandomGenerator.h"

class ShuffleRandomGenerator : public RandomGenerator {
protected:
    unsigned int size;
    std::vector<int> array;
    unsigned int curr_idx;
    std::default_random_engine rng = std::default_random_engine {};
public:
    void resetGenerator (int min_range, int max_range) override {
        size = (unsigned int) max_range - min_range + 1;
        array = std::vector<int> (size);
        curr_idx = 0;
        std::iota (std::begin(array), std::end(array), 0);
        std::shuffle (array.begin(), array.end(), rng);
    }

    int getRandomNumber () override {
        if (curr_idx == size) {
            curr_idx = 0;
            std::shuffle (array.begin(), array.end(), rng);
        }

        return array[curr_idx++];
    }

    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
        for (int i = 0; i < sample_size; i++) {
            sample[i] = getRandomNumber();
        }
    }

};

#endif //USAC_SHUFFLERANDOMGENERATOR_H
