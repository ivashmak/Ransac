#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

class RandomGenerator {
protected:
    unsigned int sample_size;
public:
    void resetTime () {
        srand (time(NULL));
    }
    void setSampleSize (unsigned int sample_size) {
        this->sample_size = sample_size;
    }

    virtual int getRandomNumber () = 0;
    virtual void generateUniqueRandomSample (int * sample) = 0;
    virtual void resetGenerator (int min_range, int max_range) = 0;
};
#endif //USAC_RANDOMGENERATOR_H
