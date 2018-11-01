#include "GraphCut.h"

void GraphCut::labeling (const int * const neighbors, Estimator * estimator, Model * model, int * inliers, int points_size) {
    int knn = model->k_nearest_neighbors;
    float lambda = model->lambda_graph_cut;
    Energy<float, float, float> *e = new Energy<float, float, float>(points_size,
                                                                     knn * points_size,
                                                                     NULL);

    for (auto i = 0; i < points_size; ++i) {
        e->add_node();
    }

    const float sqr_thr = 2 * model->threshold * model->threshold;
    for (int i = 0; i < points_size; ++i) {
        float distance = estimator->GetError(i);
        float energy = exp(-(distance * distance) / sqr_thr);

        e->add_term1(i, energy, 0);
    }

    float distance1, distance2, energy1, energy2;
    int neighbor_idx;
    for (int i = 0; i < points_size; ++i) {
        distance1 = estimator->GetError(i);
        energy1 = exp(-(distance1 * distance1) / sqr_thr);

        neighbor_idx = knn * i;
        for (int j = 0; j < knn; ++j) { // j = 1, neighbors is reduced by first neighbor (itself)
            int n_idx = neighbors[neighbor_idx + j];

            distance2 = estimator->GetError(n_idx);
            energy2 = exp(-(distance2 * distance2) / sqr_thr);

            const float e00 = 0.5 * (energy1 + energy2);
            const float e01 = 1;
            const float e10 = 1;
            const float e11 = 1 - e00; // 1 - 0.5 * (energy1 + energy2);

            if (e00 + e11 > e01 + e10) {
                std::cout << "\033[1;33m"
                             "Non-submodular expansion term detected;"
                             "smooth costs must be a metric for expansion  \033[0m\n";
            }

            e->add_term2(i, n_idx, e00 * lambda, e01 * lambda, e10 * lambda, e11 * lambda);
        }
    }

    e->minimize();
    unsigned int inl = 0;
    for (int i = 0; i < points_size; ++i) {
        if (e->what_segment(i) == Graph<float, float, float>::SINK) {
            inliers[inl++] = i;
        }
    }

    delete e;
}