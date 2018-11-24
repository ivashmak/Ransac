#include "GraphCut.h"

/*
 * In expirements I found that for less k nearest neighbors
 * the number inliers is more. For 2d line fitting.
 */
void GraphCut::labeling (const int * const neighbors, Estimator * estimator, 
    Model * model, int * inliers, Score &score, int points_size, bool get_inliers) {

    estimator->setModelParameters(model->returnDescriptor());

    int knn = model->k_nearest_neighbors;
    float lambda = model->lambda_graph_cut;
    Energy<float, float, float> *e = new Energy<float, float, float>(points_size, knn * points_size, NULL);

    for (auto i = 0; i < points_size; ++i) {
        e->add_node();
    }

    float * errors = new float [points_size];

    const float sqr_thr = 2 * model->threshold * model->threshold;
    float energy, distance;
    for (int i = 0; i < points_size; ++i) {
        distance = estimator->GetError(i);

        // save errors to avoid next error calculating
        errors[i] = distance;
        
        energy = exp(-(distance * distance) / sqr_thr);

        e->add_term1(i, energy, 0);
    }

    // std::cout << "add terms2\n";

    float distance1, distance2, energy1, energy2;
    int neighbors_row, n_idx;
    float e00, e01, e10, e11;
    for (int i = 0; i < points_size; ++i) {
        distance1 = errors[i];

        energy1 = exp(-(distance1 * distance1) / sqr_thr);

        neighbors_row = knn * i;
        for (int j = 0; j < knn; ++j) { // j = 1, neighbors are reduced by first neighbor (itself)
            
            n_idx = neighbors[neighbors_row + j];

            distance2 = errors[n_idx];
            
            energy2 = exp(-(distance2 * distance2) / sqr_thr);

            e00 = 0.5 * (energy1 + energy2);
            e01 = 1;
            e10 = 1;
            e11 = 1 - e00; // 1 - 0.5 * (energy1 + energy2);

            if (e00 + e11 > e01 + e10) {
                std::cout << "\033[1;33m"
                             "Non-submodular expansion term detected;"
                             "smooth costs must be a metric for expansion  \033[0m\n";
            }

            e->add_term2(i, n_idx, e00 * lambda, e01 * lambda, e10 * lambda, e11 * lambda);
        }
    }

    e->minimize();
    score.inlier_number = 0;
    
    if (get_inliers) {
        for (int i = 0; i < points_size; ++i) {
            if (e->what_segment(i) == Graph<float, float, float>::SINK) {
                inliers[score.inlier_number++] = i;
            }
        }
    } else {
        for (int i = 0; i < points_size; ++i) {
            if (e->what_segment(i) == Graph<float, float, float>::SINK) {
                score.inlier_number++;
            }
        }
    }

    score.score = score.inlier_number;

    delete e;
}