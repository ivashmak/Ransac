#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

#include "../../RandomGenerator/RandomGenerator.h"

class Sampler {
protected:
    unsigned int k_iterations = 0;
    unsigned int sample_size = 0;
    unsigned int points_size = 0;
public:

    /*
     * generate sample. Considering that all parameters are defined (including sample_size, points_size and
     * random generator)
     */
    virtual void generateSample (int *sample) = 0;

    /*
     * if we don't want to call setSampleSize (), we can call generateSample () with sample_size parameter
     */
    void generateSample (int *sample, unsigned int sample_size) {
        this->sample_size = sample_size;
        generateSample(sample);
    }

    /*
     * if we don't want to call setSampleSize () and setPointsSize (), we can call generateSample ()
     * with sample_size and points_size parameters
     */
    void generateSample (int *sample, unsigned int sample_size, unsigned int points_size) {
        this->sample_size = sample_size;
        this->points_size = points_size;
        generateSample(sample);
    }

    /*
     * sample size setter
     * Call random generator initialization to set subset size
     */
    void setSampleSize (unsigned int sample_size) {
        this->sample_size = sample_size;
    }

    /*
     * points size setter
     * Call random generator initialization.
     */
    void setPointsSize (unsigned int points_size) {
        this->points_size = points_size;
    }

    /*
     * Initialize randomGenerator.
     * Can be override in child class.
     * Can be empty for some child classes that does not use random generators.
     * Random Generators has function resetGenerator (int min, int max)
     * It is enough to know points size to reset generator.
     */
    virtual void initRandomGenerator () {}


    /*
     * Returns count of Sampler calls
     */
    unsigned int getNumberOfIterations () {
        return k_iterations;
    }

    /*
     * Check if sampler is safe to use and everything is initialized
     * Can be different for child classes
     */
    virtual bool isInit ()  { return false; }

};

#endif //RANSAC_SAMPLER_H