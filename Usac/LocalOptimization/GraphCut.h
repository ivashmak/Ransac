#ifndef USAC_GRAPHCUT_H
#define USAC_GRAPHCUT_H

#include <opencv2/core/mat.hpp>
#include "../Estimator/Estimator.h"
#include "../../include/gco-v3.0/GCoptimization.h"
#include "LocalOptimization.h"

class GraphCut : public LocalOptimization {
protected:
    float threshold;
    unsigned int points_size;
    Estimator * estimator;
    int knn;
    float lambda;
    float sqr_thr;
    int * neighbors;
public:
    void init (unsigned int points_size_, Model * model, Estimator * estimator_, const int * const neighbors_) {
        lambda = model->lambda_graph_cut;
        knn = model->k_nearest_neighbors;
        threshold = model->threshold;
        estimator = estimator_;
        sqr_thr = 2 * threshold * threshold;
        points_size = points_size_;
        neighbors = const_cast<int *>(neighbors_);
    }

	void labeling (const cv::Mat& model, Score * score, bool get_inliers=false, int * inliers = nullptr);
};

#endif //USAC_GRAPHCUT_H
