#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

class RandomGenerator {
protected:
    unsigned int sample_size;
public:

    virtual unsigned int getRandomNumber () = 0;

    void setSampleSize (unsigned int sample_size) {
        this->sample_size = sample_size;
    }

    virtual void generateRandomSample (int * sample) = 0;

};
#endif //USAC_RANDOMGENERATOR_H
