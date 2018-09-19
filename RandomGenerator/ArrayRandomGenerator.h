#ifndef USAC_ARRAYRANDOMGENERATOR_H
#define USAC_ARRAYRANDOMGENERATOR_H

#include <cstdlib>
#include <cmath>
#include <iostream>
#include "RandomGenerator.h"

/*
 *
 *
 */
class ArrayRandomGenerator : public RandomGenerator {
protected:
    int *array;
    int max;
    unsigned int array_size;
public:

    /*
     * if min = 0 and max = 5
     * Array will be [0,1,2,3,4,5]
     * Size is max-min+1
     */
    void resetGenerator (int min_range, int max_range) override {
        array_size = (unsigned int) max_range - min_range + 1;
        max = array_size;

        array = new int[array_size];

        int k = 0;
        for (int i = min_range; i <= max_range; i++) {
            array[k] = i;
            k++;
        }
    }

    int getRandomNumber () override {
        if (max == 0) max = array_size;

        unsigned int random_number = (unsigned int) random () % max;

        int temp = array[random_number];
        max--;
        array[random_number] = array[max];
        array[max] = temp;

        return temp;
    }

    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
        for (int i = 0; i < sample_size; i++) {
            sample[i] = getRandomNumber();
        }
    }



};

#endif //USAC_ARRARANDOMGENERATOR_H
