// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_SAMPLER_H
#define RANSAC_SAMPLER_H

#include "random_generator.hpp"

namespace cv { namespace usac {
class Sampler {
protected:
    unsigned int k_iterations = 0, points_size = 0, sample_size = 0;
public:
    virtual ~Sampler() = default;

    /*
     * generate sample. Considering that all parameters are defined (including sample_size, points_size and
     * random generator)
     */
    virtual void generateSample(int *sample) = 0;

    /*
     * Returns number of iterations
     */
    unsigned int getNumberOfIterations() {
        return k_iterations;
    }

    /*
     * Check if sampler is safe to use and everything is initialized
     * Can be different for child classes
     */
    virtual bool isInit() { return false; }
};
}}
#endif //RANSAC_SAMPLER_H