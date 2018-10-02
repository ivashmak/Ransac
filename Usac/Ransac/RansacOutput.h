#ifndef USAC_RANSACOUTPUT_H
#define USAC_RANSACOUTPUT_H

#include "../Model.h"

struct Time {
    long minutes;
    long seconds;
    long milliseconds;
    long microseconds;
};

class RansacOutput {
private:
    Model * model;
    Model * non_minimal_model;
    Time * time;
    std::vector<int> inliers;
    long time_mcs;
    unsigned int number_inliers;
    unsigned int number_iterations;
public:

    ~RansacOutput() {
        delete model, non_minimal_model, time;
    }

    RansacOutput (const Model * const model_,
                  const Model * const non_minimal_model_,
                  const std::vector<int>& inliers_,
                  long time_mcs_,
                  unsigned int number_inliers_,
                  unsigned int number_iterations_) {

        /*
         * Let's make a deep copy to avoid changing variables from origin input.
         * And make them changeable for further using.
         */
        model = new Model (*model_);
        non_minimal_model = new Model (*non_minimal_model_);
        inliers = inliers_;
        time_mcs = time_mcs_;
        number_inliers = number_inliers_;
        number_iterations = number_iterations_;

        time = new Time;
        time->microseconds = time_mcs % 1000;
        time->milliseconds = ((time_mcs - time->microseconds)/1000) % 1000;
        time->seconds = ((time_mcs - 1000*time->milliseconds - time->microseconds)/(1000*1000)) % 60;
        time->minutes = ((time_mcs - 60*1000*time->seconds - 1000*time->milliseconds - time->microseconds)/(60*1000*1000)) % 60;

    }

    void printTime () {
        std::cout << time->seconds << " secs, " << time->milliseconds << " ms, " << time->microseconds << " mcs\n";
    }

    std::vector<int> getInliers () {
        return inliers;
    }

    long getTimeMicroSeconds () {
        return time_mcs;
    }

    unsigned int getNumberOfInliers () {
        return number_inliers;
    }

    unsigned int getNumberOfIterations () {
        return number_iterations;
    }

    Time* getTime () {
        return time;
    }

    Model* getModel () {
        return model;
    }

    Model* const getNonMinimalModel () {
        return non_minimal_model;
    }

};

#endif //USAC_RANSACOUTPUT_H
