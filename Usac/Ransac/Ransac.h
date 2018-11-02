#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "../Estimator/Estimator.h"
#include "../Estimator/Line2DEstimator.h"
#include "../Quality/Quality.h"
#include "../Sampler/Sampler.h"

#include "../Verbose.h"
#include "RansacOutput.h"

class Ransac {
protected:
    /*
     * Initialize them to 0 to check if they are null
     */
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;
    RansacOutput * ransac_output;
    Estimator * estimator;

    cv::Mat knn_neighbors;
    int * neighbors = nullptr;
public:
    
    ~Ransac () {}

    Ransac (Model &model,
            Sampler &sampler,
            TerminationCriteria &termination_criteria,
            Quality &quality,
            Estimator &estimator) {

        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = &quality;
        this->estimator = &estimator;
    }

    void set_neighbors (const cv::Mat& neighbors_) {
//        knn_neighbors = neighbors_.clone();
        neighbors = (int *) neighbors_.data;
    }

    const int * get_neighbors () const {
        assert(neighbors != nullptr);
        return neighbors;
    }

    void run (cv::InputArray input_points, bool LO=false);

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
