// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "init.hpp"

#include "../estimator/estimator.hpp"
#include "../quality/quality.hpp"
#include "../sampler/sampler.hpp"
#include "ransac_output.hpp"
#include "../sprt.hpp"
#include "../local_optimization/local_optimization.hpp"
#include "../utils/nearest_neighbors.hpp"

class Ransac {
protected:
    /*
     * Initialize them to 0 to check if they are null
     */
    Model * model;
    Quality * quality;
    Sampler * sampler;
    TerminationCriteria * termination_criteria;
    RansacOutput * ransac_output;
    Estimator * estimator;
    LocalOptimization * local_optimization;
    SPRT * sprt;

    cv::Mat neighbors_m;
    std::vector<std::vector<int>> neighbors_v;
    unsigned int points_size;
public:
    
    ~Ransac () {
        if (model->sprt) delete (sprt);
        if (model->lo != LocOpt ::NullLO) delete (local_optimization);
        delete(sampler); delete (quality); delete (estimator); delete(termination_criteria);
    }

    Ransac (Model * model_, cv::InputArray points) {
        model = model_;

        assert(! points.empty());
        assert(model != nullptr);

        points_size = points.getMat().rows;
//        std::cout << "points size = " << points_size << "\n";

        initEstimator (estimator, model->estimator, points.getMat());
        initSampler (sampler, model, points.getMat());

        // Init quality
        quality = new Quality;
        quality->init(points_size, model->threshold, estimator);
        //

        initLocalOptimization(local_optimization, model, estimator, quality, points_size);

        // Get neighbors
        cv::Mat neighbors_dists;
        if (model->sampler == SAMPLER::Napsac || model->lo == LocOpt::GC) {
            if (model->neighborsType == NeighborsSearch::Grid) {
                NearestNeighbors::getGridNearestNeighbors(points.getMat(), model->cell_size, neighbors_v);
                if (model->sampler == SAMPLER::Napsac) {
                    ((NapsacSampler *) sampler)->setNeighbors(neighbors_v, NeighborsSearch::Grid);
                } else {
                    ((GraphCut *) local_optimization)->setNeighbors(neighbors_v, NeighborsSearch::Grid);
                }
            } else {
                NearestNeighbors::getNearestNeighbors_nanoflann(points.getMat(), model->k_nearest_neighbors, neighbors_m, false, neighbors_dists);
                if (model->sampler == SAMPLER::Napsac) {
                    ((NapsacSampler *) sampler)->setNeighbors(neighbors_m, NeighborsSearch::Nanoflann);
                } else {
                    ((GraphCut *) local_optimization)->setNeighbors(neighbors_m, NeighborsSearch::Nanoflann);
                }
            }
        }
        // -----------------

        // init termination criteria
        if (model->sampler == SAMPLER::Prosac) {
            initProsacTerminationCriteria(termination_criteria, sampler, model, estimator, points_size);
        } else {
            initTerminationCriteria(termination_criteria, model, points_size);
        }
        // ----------------------------

        // Init SPRT
        if (model->sprt) {
            sprt = new SPRT (model, estimator, points_size);
        }
    }

//    Ransac (Model * model_,
//            Sampler * sampler_,
//            TerminationCriteria * termination_criteria_,
//            Quality * quality_,
//            Estimator * estimator_) {
//
//        assert (model_ != nullptr);
//        assert (sampler_ != nullptr);
//        assert (termination_criteria_ != nullptr);
//        assert (quality_ != nullptr);
//        assert (estimator_ != nullptr);
//
//        model = model_;
//        sampler = sampler_;
//        termination_criteria = termination_criteria_;
//        quality = quality_;
//        estimator = estimator_;
//    }

    void run ();

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
