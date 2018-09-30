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
public:
    void resetGenerator (int min_range, int max_range) override {
        array_size = (unsigned int) max_range - min_range + 1;
        array = new int[array_size];
        curr_idx = 0;

        int k = 0;
        for (int i = min_range; i <= max_range; i++) {
            array[k++] = i;
        }

        shuffleArray();
    }

    void shuffleArray () {
        int max = array_size;
        for (int i = 0; i < array_size; i++) {
            unsigned int random_number = (unsigned int) random () % max;

            int temp = array[random_number];
            max--;
            array[random_number] = array[max];
            array[max] = temp;
        }
    }

    int getRandomNumber () override {
        if (curr_idx == array_size) {
            curr_idx = 0;
            shuffleArray();
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
