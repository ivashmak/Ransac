#include "GraphCut.h"
#include <math.h>

void GraphCut::labeling (const cv::Mat& model, Score * score, int * inliers) {

    estimator->setModelParameters(model);

    Energy<float, float, float> * e = new Energy<float, float, float>(points_size, knn * points_size, NULL);

    for (auto i = 0; i < points_size; ++i) {
        e->add_node();
    }

    float energy, distance;

    score->inlier_number = 0;

    for (int i = 0; i < points_size; ++i) {
        distance = estimator->GetError(i);
        // save errors to avoid next error calculating
        errors[i] = distance;

        energy = exp(-(distance * distance) / sqr_thr);

        e->add_term1(i, energy, 0);
    }


    float distance1, distance2, energy1, energy2;
    int neighbors_row, n_idx;
    float e00, e01, e10, e11;
    if (neighborsType == NeighborsSearch::Nanoflann) {
        for (int i = 0; i < points_size; ++i) {
            distance1 = errors[i];

            energy1 = exp(-(distance1 * distance1) / sqr_thr);

            neighbors_row = knn * i;
            for (int j = 0; j < knn; ++j) { // j = 1, neighbors are reduced by first neighbor (itself)

                n_idx = neighbors[neighbors_row + j];

                distance2 = errors[n_idx];

                if (n_idx == i) {
                    //                std::cout << "\033[1;31mIndex of neighbor is equal to index of point. Continue.\033[0m \n";
                    continue;
                }

                energy2 = exp(-(distance2 * distance2) / sqr_thr);

                e00 = 0.5 * (energy1 + energy2);
                e01 = 1;
                e10 = 1;
                e11 = 1 - e00; // 1 - 0.5 * (energy1 + energy2);

                // B -= A; C -= D;
                // assert(B + C >= 0); /* check regularity */
                // e01*l - e00*l + e10*l - e11*l >= 0
                // l*(e01 - e00 + e10 - e11) >= 0
                // l > 0 => (e01 - e00 + e10 - e11) >= 0
                // e01 + e10 >= e00 + e11
                if (e00 + e11 > e01 + e10 || std::isnan(e00)) {
                    //                std::cout << "\033[1;31m"
                    //                             "Non-submodular expansion term detected;"
                    //                             "smooth costs must be a metric for expansion  \033[0m\n";
                    continue;
                }
                e->add_term2(i, n_idx, e00 * lambda, e01 * lambda, e10 * lambda, e11 * lambda);
            }
        }
    } else {
        for (int i = 0; i < points_size; ++i) {
            distance1 = errors[i];
            energy1 = exp(-(distance1 * distance1) / sqr_thr);
//            std::cout << neighbors_v[i].size() << " = neighbors size\n";
            for (int j = 0; j < neighbors_v[i].size(); ++j) {
                n_idx = neighbors_v[i][j];
                distance2 = errors[n_idx];
                energy2 = exp(-(distance2 * distance2) / sqr_thr);
                e00 = 0.5 * (energy1 + energy2);
                e01 = 1;
                e10 = 1;
                e11 = 1 - e00;
                if (e00 + e11 > e01 + e10 || std::isnan (e00)) {
//                std::cout << "\033[1;31m"
//                             "Non-submodular expansion term detected;"
//                             "smooth costs must be a metric for expansion  \033[0m\n";
                    continue;
                }
                e->add_term2(i, n_idx, e00 * lambda, e01 * lambda, e10 * lambda, e11 * lambda);
            }
        }
    }

    e->minimize();

    score->inlier_number = 0;
    for (int i = 0; i < points_size; ++i) {
        if (e->what_segment(i) == Graph<float, float, float>::SINK) {
            inliers[score->inlier_number++] = i;
        }
    }
    score->score = score->inlier_number;

    delete e;
}
