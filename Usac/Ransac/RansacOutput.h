#ifndef USAC_RANSACOUTPUT_H
#define USAC_RANSACOUTPUT_H

#include "../Model.h"
#include "../Utils/Math.h"

class RansacOutput {
private:
    Model * model;
    Time * time;
    std::vector<int> inliers;
    long time_mcs;
    unsigned int number_inliers;
    unsigned int number_iterations;

    unsigned int lo_inner_iters;
    unsigned int lo_iterative_iters;
    unsigned int gc_iters;
public:

   ~RansacOutput() {
       delete model, time;
   }

    RansacOutput (const Model * const model_,
                  const int * const inliers_,
                  long time_mcs_,
                  unsigned int number_inliers_,
                  unsigned int number_iterations_,
                  unsigned int lo_inner_iters_,
                  unsigned int lo_iterative_iters_,
                  unsigned int gc_iters_) {

        /*
         * Let's make a deep copy to avoid changing variables from origin input.
         * And make them changeable for further using.
         */
    
        model = new Model (*model_);        
        inliers.assign (inliers_, inliers_ + number_inliers_);
        time_mcs = time_mcs_;
        number_inliers = number_inliers_;
        number_iterations = number_iterations_;

        lo_inner_iters = lo_inner_iters_;
        lo_iterative_iters = lo_iterative_iters_;
        gc_iters = gc_iters_;

        time = new Time;
        splitTime (time, time_mcs);
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

    unsigned int getNumberOfMainIterations () {
        // number_iterations > number_lo_iterations
        return number_iterations;
    }

    unsigned int getLOIters () {
        // number_iterations > number_lo_iterations
        return lo_inner_iters + lo_iterative_iters + gc_iters;
    }
    
    unsigned int getLOInnerIters () {
        return lo_inner_iters;
    }

    unsigned int getLOIterativeIters () {
        return lo_iterative_iters;
    }

    unsigned int getGCIters () {
        return gc_iters;
    }

    Time* getTime () {
        return time;
    }

    Model* getModel () {
        return model;
    }

};

#endif //USAC_RANSACOUTPUT_H
