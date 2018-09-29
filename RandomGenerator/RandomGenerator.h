#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

#include <chrono>

class RandomGenerator {
protected:
    unsigned int subset_size = 0;
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

    virtual void generateUniqueRandomSet (int * sample) = 0;

    void generateUniqueRandomSet (int * sample, unsigned int subset_sz) {
        subset_size = subset_sz;
        generateUniqueRandomSet(sample);
    }

    void generateUniqueRandomSet (int * sample, unsigned int subset_sz, int size) {
        resetGenerator(0, size);
        subset_size = subset_sz;
        generateUniqueRandomSet(sample);
    }

    void generateUniqueRandomSet (int * sample, unsigned int subset_sz, int min, int max) {
        resetGenerator(min, max);
        subset_size = subset_sz;
        generateUniqueRandomSet(sample);
    }

    void setSubsetSize (unsigned int subset_sz) {
        subset_size = subset_sz;
    }

    virtual bool isInit() {
        return false;
    }

};
#endif //USAC_RANDOMGENERATOR_H
