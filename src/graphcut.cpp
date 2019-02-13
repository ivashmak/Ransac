// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "graphcut.hpp"

void GraphCut::labeling (const cv::Mat& model, Score * score, int * inliers) {

    estimator->setModelParameters(model);

    Energy<float, float, float> * e = new Energy<float, float, float>(points_size, neighbor_number, NULL);

    for (auto i = 0; i < points_size; ++i) {
        e->add_node();
    }

    float energy, energy1, energy2, distance, distance1, distance2;

    for (unsigned int i = 0; i < points_size; ++i) {
        distance = estimator->GetError(i);
        // save errors to avoid next error calculating
        errors[i] = distance;

        energy = exp(-(distance * distance) / sqr_thr);

        e->add_term1(i, energy, 0);
    }


    unsigned int neighbors_row, n_idx;
    float e00, e01 = 1, e10 = 1, e11;
    if (neighborsType == NeighborsSearch::Nanoflann) {
        for (unsigned int i = 0; i < points_size; ++i) {
            distance1 = errors[i];

            energy1 = exp(-(distance1 * distance1) / sqr_thr);

            neighbors_row = knn * i;
            for (unsigned int j = 0; j < knn; ++j) {

                n_idx = neighbors[neighbors_row + j];

                distance2 = errors[n_idx];

                if (n_idx == i) {
                    //  std::cout << "\033[1;31mIndex of neighbor is equal to index of point. Continue.\033[0m \n";
                    continue;
                }

                energy2 = exp(-(distance2 * distance2) / sqr_thr);

                e00 = (energy1 + energy2) / 2;
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
                e->add_term2(i, n_idx, e00 * spatial_coherence, e01 * spatial_coherence, e10 * spatial_coherence, e11 * spatial_coherence);
            }
        }
    } else {
        unsigned int neighbors_i_size;
        for (unsigned int i = 0; i < points_size; ++i) {
            distance1 = errors[i];
            energy1 = exp(-(distance1 * distance1) / sqr_thr);
            neighbors_i_size = neighbors_v[i].size();
            for (unsigned int j = 0; j < neighbors_i_size; ++j) {
                n_idx = neighbors_v[i][j];
                distance2 = errors[n_idx];
                energy2 = exp(-(distance2 * distance2) / sqr_thr);
                e00 = (energy1 + energy2) / 2;
                e11 = 1 - e00;
                if (e00 + e11 > e01 + e10 || std::isnan (e00)) {
                    continue;
                }
                e->add_term2(i, n_idx, e00 * spatial_coherence, e01 * spatial_coherence, e10 * spatial_coherence, e11 * spatial_coherence);
            }
        }
    }

    e->minimize();

    score->inlier_number = 0;
    for (unsigned int i = 0; i < points_size; ++i) {
        if (e->what_segment(i) == Graph<float, float, float>::SINK) {
            inliers[score->inlier_number++] = i;
        }
    }
    score->score = score->inlier_number;

    delete e;
}
