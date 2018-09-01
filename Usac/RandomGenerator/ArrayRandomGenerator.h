#ifndef USAC_ARRAYRANDOMGENERATOR_H
#define USAC_ARRAYRANDOMGENERATOR_H

#include "RandomGenerator.h"

class ArrayRandomGenerator : public RandomGenerator {
protected:
    unsigned int *array;
    unsigned int max;
    unsigned int N_points;
    unsigned int sample_size;
public:

    void resetGenerator (int min_range, int max_range) override {
        this->N_points = (uint) max_range - min_range;

        if (!array) {
            array = new unsigned int[N_points];

            for (unsigned int i = 0; i < N_points; i++) {
                array[i] = i;
            }
        } else {
            reallocArray ();
        }

    }

    int getRandomNumber () override {
        max = max == 0 ? N_points : max;

        unsigned int random_number = rand () % max;


        unsigned int temp = array[random_number];
        array[random_number] = array[max-1];
        array[max-1] = temp;
        max--;

        return temp;
    }

    void reallocArray () {
        array = (unsigned int *) realloc(array, sizeof (unsigned int) * N_points);
        for (unsigned int i = 0; i < N_points; i++) {
            array[i] = i;
        }
    }

    void generateUniqueRandomSample (int * sample) override {
        for (int i = 0; i < sample_size; i++) {
            sample[i] = getRandomNumber();
        }
    }

};

#endif //USAC_ARRARANDOMGENERATOR_H
