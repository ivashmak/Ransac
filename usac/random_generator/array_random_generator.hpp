#ifndef USAC_ARRAYRANDOMGENERATOR_H
#define USAC_ARRAYRANDOMGENERATOR_H

#include "random_generator.hpp"

/*
 * Fischer-Yates Shuffle
 */
class ArrayRandomGenerator : public RandomGenerator {
protected:
    unsigned int array_size = 0;
    int * array;
    unsigned int max;
public:

    virtual ~ArrayRandomGenerator () {
         if (isInit()) delete[] array;
    }

    // range in closed interval <min; max>
    void resetGenerator (int min, int max) override {
        array_size = max - min + 1;
        max = array_size;
        array = new int[array_size];
        int k = 0;
        for (int i = min; i <= max; i++) {
            array[k++] = i;
        }
    }

    int getRandomNumber () override {
        if (max == 0) {
            max = array_size;
        }
        unsigned int array_random_index = (unsigned int) random () % max;

        int random_number = array[array_random_index];
        max--;
        array[array_random_index] = array[max];
        array[max] = random_number;

        return random_number;
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

#endif //USAC_ARRARANDOMGENERATOR_H
