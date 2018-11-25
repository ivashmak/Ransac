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
    unsigned int array_size = 0;
    int * array;
    unsigned int curr_idx;
    int MIN = 1000000, MAX = 0;
public:

    virtual ~ArrayRandomGenerator () {
         if (isInit()) delete array;
    }

    void resetGenerator (int min_range, int max_range) override {
        array_size = max_range - min_range + 1;
            
        if (MIN > min_range || max_range > MAX) {
            if (MIN > min_range) {
                MIN = min_range;
            }
            if (MAX < max_range) {
                MAX = max_range;
            }
            array = new int[array_size];
        }

        curr_idx = 0;
        int k = 0;
        for (int i = min_range; i <= max_range; i++) {
            array[k++] = i;
        }
        shuffleArray();
    }

    void shuffleArray () {
//        std::cout << "shuffle array\n";
        int max = array_size;
        for (int i = 0; i < array_size; i++) {
            unsigned int array_random_index = (unsigned int) random () % max;

            int temp = array[array_random_index];
            max--;
            array[array_random_index] = array[max];
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
        // If requested subset size is bigger than array size,
        // so there will be shuffleArray (). But after shuffleArray
        // subset can not be Unique. So we should shuffle array before that.
        if (curr_idx + subset_size > array_size) {
            shuffleArray ();
        }

        for (int i = 0; i < subset_size; i++) {
            sample[i] = getRandomNumber();
        }
    }

    bool isInit () override {
        return subset_size != 0 && array_size != 0;
    }

};

#endif //USAC_ARRARANDOMGENERATOR_H
