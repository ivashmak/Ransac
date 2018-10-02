#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "../Estimator/Estimator.h"
#include "../Estimator/Line2DEstimator.h"
#include "../Quality.h"
#include "../Sampler/Sampler.h"

#include "../Verbose.h"
#include "RansacOutput.h"

class Ransac {
protected:
    /*
     * Initialize them to 0 to check if they are null
     */
    Model *model = 0;
    Quality *quality = 0;
    Sampler *sampler = 0;
    TerminationCriteria *termination_criteria = 0;
    RansacOutput * ransac_output;
public:

    Ransac (Model &model,
            Sampler &sampler,
            TerminationCriteria &termination_criteria,
            Quality &quality) {

        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = &quality;
    }

    void run (cv::InputArray input_points, Estimator * const estimator2d, bool LO=false);

    void setSampler (Sampler &sampler) {
        this->sampler = &sampler;
    }

    void setModel (Model &model) {
        this->model = &model;
    }

    void setTerminationCriteria (TerminationCriteria &termination_criteria) {
        this->termination_criteria = &termination_criteria;
    }

    void setQuality (Quality& quality) {
        this->quality = &quality;
    }

    RansacOutput* getRansacOutput () {
        return ransac_output;
    }

};



#endif //RANSAC_RANSAC_H
