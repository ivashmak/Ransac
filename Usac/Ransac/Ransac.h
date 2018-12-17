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

    int * neighbors = nullptr;
    std::vector<std::vector<int>> neighbors_v;
public:
    
    ~Ransac () {}

    Ransac (Model * model_,
            Sampler * sampler_,
            TerminationCriteria * termination_criteria_,
            Quality * quality_,
            Estimator * estimator_) {

        assert (model_ != nullptr);
        assert (sampler_ != nullptr);
        assert (termination_criteria_ != nullptr);
        assert (quality_ != nullptr);
        assert (estimator_ != nullptr);

        model = model_;
        sampler = sampler_;
        termination_criteria = termination_criteria_;
        quality = quality_;
        estimator = estimator_;
    }

    void setNeighbors (const cv::Mat& neighbors_) {
//        knn_neighbors = neighbors_.clone();
//        std::cout << neighbors_ << "\n\n";
        assert(!neighbors_.empty());
        neighbors = (int *) neighbors_.data;
    }

    void setNeighbors (const std::vector<std::vector<int>>& neighbors_) {
        assert(!neighbors_.empty());
        neighbors_v = neighbors_;
    }

    void run (cv::InputArray input_points);

    void setSampler (Sampler * sampler) {
        this->sampler = sampler;
    }

    void setModel (Model * model) {
        this->model = model;
    }

    void setTerminationCriteria (TerminationCriteria *termination_criteria) {
        this->termination_criteria = termination_criteria;
    }

    void setQuality (Quality * quality) {
        this->quality = quality;
    }

    RansacOutput* getRansacOutput () {
        return ransac_output;
    }

};



#endif //RANSAC_RANSAC_H
