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

    void resetGenerator (int min_range, int max_range) override {
        array_size = (unsigned int) max_range - min_range;
        max = array_size;

        array = new int[array_size];

        int k = 0;
        for (int i = min_range; i < max_range; i++) {
            array[k] = i;
            k++;
        }
    }

    int getRandomNumber () override {
        max = max == 0 ? array_size : max;

        unsigned int random_number = (unsigned int) random () % max;

        std::cout << random_number << '\n';

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

    void calculateEntropy (int size) {
        resetGenerator(0, size);

        std::cout << "gened\n";

        int * histogram = new int [size];
        for (int i = 0; i < size; i++) {
            int r = getRandomNumber();
            std::cout << r << '\n';
            histogram[r]++;
        }

        float E = 0;
        for (int i = 0; i < size; i++) {
            E -= ((float)histogram[i]/size) * log ((float) histogram[i]/size);
        }

        std::cout << size << '\n';
        double max = -log ((float) 1/size);
        std::cout << "Entropy (min: 0, max " << max << "): " << E << '\n';
    }

};

#endif //USAC_ARRARANDOMGENERATOR_H
