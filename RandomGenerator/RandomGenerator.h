#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

#include <chrono>

class RandomGenerator {
public:
    void resetTime () {
        srand (time(NULL));
    }

    virtual int getRandomNumber () = 0;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!!!
     * random numbers will be from min_range to max_range (including bounds)
     * for example min = 0, max = 5, random_numbers = {0,1,2,3,4,5}
     */
    virtual void resetGenerator (int min_range, int max_range) = 0;

    virtual void generateUniqueRandomSet (int * sample, unsigned int subset_size) = 0;

    void generateUniqueRandomSet (int * sample, unsigned int subset_size, int size) {
        resetGenerator(0, size);
        generateUniqueRandomSet(sample, subset_size);
    }
    void generateUniqueRandomSet (int * sample, unsigned int subset_size, int min, int max) {
        resetGenerator(min, max);
        generateUniqueRandomSet(sample, subset_size);
    }

};
#endif //USAC_RANDOMGENERATOR_H
