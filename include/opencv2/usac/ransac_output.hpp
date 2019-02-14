// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_RANSACOUTPUT_H
#define USAC_RANSACOUTPUT_H

#include "model.hpp"
#include "math.hpp"

namespace cv { namespace usac {
class RansacOutput {
private:
    Model *model;
    Time *time;
    std::vector<int> inliers;
    long time_mcs;
    unsigned int number_inliers;
    unsigned int number_iterations;

    unsigned int lo_iterations;
public:

    ~RansacOutput() {
        delete (model);
        delete (time);
    }

    RansacOutput(const Model *const model_,
                 const int *const inliers_,
                 long time_mcs_,
                 unsigned int number_inliers_,
                 unsigned int number_iterations_,
                 unsigned int lo_iters) {

        /*
         * Let's make a deep copy to avoid changing variables from origin input.
         * And make them changeable for further using.
         */

        model = new Model(*model_);
        inliers.assign(inliers_, inliers_ + number_inliers_);
        time_mcs = time_mcs_;
        number_inliers = number_inliers_;
        number_iterations = number_iterations_;

        lo_iterations = lo_iters;

        time = new Time;
        splitTime(time, time_mcs);
    }

    std::vector<int> getInliers() {
        return inliers;
    }

    long getTimeMicroSeconds() {
        return time_mcs;
    }

    unsigned int getNumberOfInliers() {
        return number_inliers;
    }

    unsigned int getNumberOfMainIterations() {
        // number_iterations > number_lo_iterations
        return number_iterations;
    }

    unsigned int getNUmberOfLOIterarations() {
        return lo_iterations;
    }

    Time *getTime() {
        return time;
    }

    Model *getModel() {
        return model;
    }
};
}}
#endif //USAC_RANSACOUTPUT_H
