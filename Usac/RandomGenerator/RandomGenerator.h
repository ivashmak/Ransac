#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

class RandomGenerator {
public:
    void resetTime () {
        srand (time(NULL));
    }

    virtual int getRandomNumber () = 0;
    virtual void generateUniqueRandomSet (int * sample, unsigned int sample_size) = 0;
    virtual void resetGenerator (int min_range, int max_range) = 0;
};
#endif //USAC_RANDOMGENERATOR_H
