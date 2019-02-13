// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_RANDOMGENERATOR_H
#define USAC_RANDOMGENERATOR_H

namespace cv {namespace usac {
class RandomGenerator {
protected:
    unsigned int subset_size = 0;
public:
    void resetTime() {
        srand(time(NULL));
    }

    virtual int getRandomNumber() = 0;

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!!!
     * interval is closed.
     */
    virtual void resetGenerator(int min_range, int max_range) = 0;

    virtual void generateUniqueRandomSet(int *sample) = 0;

    void setSubsetSize(unsigned int subset_sz) {
        subset_size = subset_sz;
    }

    virtual bool isInit() {
        return false;
    }

};
}}
#endif //USAC_RANDOMGENERATOR_H
