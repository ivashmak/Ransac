#ifndef USAC_PRIMENUMBERRANDOMGENERATOR_H
#define USAC_PRIMENUMBERRANDOMGENERATOR_H

#include "RandomGenerator.h"

// https://github.com/preshing/RandomSequence/blob/master/randomsequence.h

class PrimeNumberRandomGenerator : public RandomGenerator{
private:
    unsigned int m_index;
    unsigned int m_intermediateOffset;

    unsigned int size;
    static unsigned int permuteQPR(unsigned int x)
    {
        static const unsigned int prime = 4294967291u;
        if (x >= prime)
            return x;  // The 5 integers out of range are mapped to themselves.
        unsigned int residue = ((unsigned long long) x*x) % prime;
        return (x <= prime / 2) ? residue : prime - residue;
    }

public:

    int getRandomNumber () override {
        return permuteQPR((permuteQPR(m_index++) + m_intermediateOffset) ^ 0x5bf03635) % size;
    }

    void resetGenerator (int min_range, int max_range) override {
        m_index = permuteQPR(permuteQPR(time(0)) + 0x682f0161);
        m_intermediateOffset = permuteQPR(permuteQPR(time(0)+1) + 0x46790905);
        size = max_range - min_range + 1;
    }

    void generateUniqueRandomSet (int * sample, unsigned int sample_size) override {
        for (unsigned int i = 0; i < sample_size; i++) {
            sample[i] = permuteQPR((permuteQPR(m_index++) + m_intermediateOffset) ^ 0x5bf03635) % size;
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                    break;
                }
            }
        }
    }


};

#endif //USAC_PRIMENUMBERRANDOMGENERATOR_H
