// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "init.hpp"

#include "estimator.hpp"
#include "quality.hpp"
#include "sampler.hpp"
#include "ransac_output.hpp"
#include "sprt.hpp"
#include "local_optimization.hpp"
#include "nearest_neighbors.hpp"

namespace cv { namespace usac {
class Ransac {
protected:
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;
    RansacOutput *ransac_output;
    Estimator *estimator;
    LocalOptimization *local_optimization;
    SPRT *sprt;

    cv::Mat neighbors_m;
    std::vector<std::vector<int>> neighbors_v;
    unsigned int points_size;
    const float *const points;
public:

    ~Ransac() {
        if (model->sprt) delete (sprt);
        if (model->lo != LocOpt::NullLO) delete (local_optimization);
        delete (sampler);
        delete (quality);
        delete (estimator);
        delete (termination_criteria);
        delete ransac_output;
    }

    Ransac(Model *model_, cv::InputArray points_) : points((float *) points_.getMat().data) {
        model = model_;

        assert(!points_.empty());
        assert(model != nullptr);
        points_size = points_.getMat().rows;

        initEstimator(estimator, model->estimator, points_.getMat());
        initSampler(sampler, model, points_.getMat());

        // Init quality
        quality = new Quality;
        quality->init(points_size, model->threshold, estimator);
        //

        initLocalOptimization(local_optimization, model, estimator, quality, points_size);

        // Get neighbors
        cv::Mat neighbors_dists;
        if (model->sampler == SAMPLER::Napsac || model->lo == LocOpt::GC) {
            if (model->neighborsType == NeighborsSearch::Grid) {
                NearestNeighbors::getGridNearestNeighbors(points_.getMat(), model->cell_size, neighbors_v);
                if (model->sampler == SAMPLER::Napsac) {
                    ((NapsacSampler *) sampler)->setNeighbors(neighbors_v, NeighborsSearch::Grid);
                } else {
                    ((GraphCut *) local_optimization)->setNeighbors(neighbors_v, NeighborsSearch::Grid);
                }
            } else {
                NearestNeighbors::getNearestNeighbors_nanoflann(points_.getMat(), model->k_nearest_neighbors,
                                                                neighbors_m, false, neighbors_dists);
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
            sprt = new SPRT(model, estimator, points_size);
        }
    }

    void run();

    RansacOutput *getRansacOutput() {
        return ransac_output;
    }
};
}}


#endif //RANSAC_RANSAC_H
