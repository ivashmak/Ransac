// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef SPRT_H
#define SPRT_H

////////////////////////////////////////////////////////////////////////////
//
//	Copyright (c) 2012 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty.
//
//	Please send BUG REPORTS to rraguram@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////

#include "precomp.hpp"

#include "model.hpp"
#include "utils/math.hpp"
#include "estimator/estimator.hpp"

#define LOG_ETA_0 log(0.05)
/*
 * Sequential Probability Ratio Test
 *
 * From:
 * Randomized RANSAC with sequential probability ratio test
 * Authors: J. Matas, O. Chum
 * https://ieeexplore.ieee.org/document/1544925
 */

class SPRT {
private:
    class SPRT_history {
    public:
        double epsilon, delta, A;
        // k is number of samples processed by test
        int k;
    };

    /*
     * The probability of a data point being consistent
     * with a ‘bad’ model is modeled as a probability of
     * a random event with Bernoulli distribution with parameter
     * δ : p(1|Hb) = δ.
     */

    /*
     * The probability p(1|Hg) = ε
     * that any randomly chosen data point is consistent with a ‘good’ model
     * is approximated by the fraction of inliers ε among the data
     * points
     */

    /*
     * The decision threshold A is the only parameter of the Adapted SPRT
     */
    // i
    unsigned int current_sprt_idx;
    int last_sprt_update;

    double t_M, m_S, threshold, confidence;
    unsigned int points_size, sample_size, max_iterations, random_pool_idx;
    std::vector<SPRT_history*> sprt_histories;

    Estimator * estimator;

    int number_rejected_models;
    int sum_fraction_data_points;
    unsigned int * points_random_pool;

    int max_hypothesis_test_before_sprt;
public:

    ~SPRT() {
        delete[] points_random_pool;
    }

    SPRT (Model * model, Estimator * estimator_, unsigned int points_size_) {
        if (model->reset_random_generator) srand(time(NULL));

        // Generate array of points
        points_random_pool = new unsigned int [points_size_];
        for (unsigned int i = 0; i < points_size_; i++) {
            points_random_pool[i] = i;
        }
        unsigned int temp;
        int max = points_size_;
        // Shuffle random pool of points.
        for (unsigned int i = 0; i < points_size_; i++) {
            random_pool_idx = (unsigned int) random () % max;
            temp = points_random_pool[random_pool_idx];
            max--;
            points_random_pool[random_pool_idx] = points_random_pool[max];
            points_random_pool[max] = temp;
        }
        random_pool_idx = 0;
        //

        sprt_histories = std::vector<SPRT_history*>();
        sprt_histories.push_back(new SPRT_history);
        estimator = estimator_;

        if (model->estimator == ESTIMATOR::Homography) {
            // t_M = 200, m_S = 1, delta0 = 0.01, epsilon0 = 0.1;
            sprt_histories[0]->delta = 0.01;
            sprt_histories[0]->epsilon = 0.1;

            // time t_M needed to instantiate a model hypotheses given a sample
            t_M = 200;
            // Let m_S be the number of models that are verified per sample
            m_S = 1;
        } else if (model->estimator == ESTIMATOR::Fundamental) {
            // t_M = 200, m_S = 2.48, delta0 = 0.05, epsilon0 = 0.2;
            sprt_histories[0]->delta = 0.05;
            sprt_histories[0]->epsilon = 0.2;
            t_M = 200;
            m_S = 2.48;
        } else if (model->estimator == ESTIMATOR::Essential) {
            // ??????????????????????????????????????????????
            sprt_histories[0]->delta = 0.05;
            sprt_histories[0]->epsilon = 0.2;
            t_M = 300;
            m_S = 4; // up to 10 models
        } else if (model->estimator == ESTIMATOR::Line2d){
            t_M = 100;
            m_S = 1;
            /*
             * The initial estimate δ0 is obtained by geometric considerations,
             * i.e. as a fraction of the area that supports a hypothesised model
             * (a strip around an epipolar line in case of epipolar geometry)
             * to the area of possible appearance of outlier data (the area of
             * the search window). Alternatively, a few models can be evaluated
             * without applying SPRT in order to obtain an initial estimate of δ.
             */
            sprt_histories[0]->delta = 0.0001; // ???

            /*
             * The initial value of ε0 can be derived from the maximal time the
             * user is willing to allocate to the algorithm.
             */
            sprt_histories[0]->epsilon = 0.001; // ???

        } else {
            std::cout << "UNDEFINED ESTIMATOR\n";
            exit (111);
        }

        current_sprt_idx = 0;
        last_sprt_update = 0;

        sample_size = model->sample_size;
        threshold = model->threshold;
        confidence = model->desired_prob;
        max_iterations = model->max_iterations;
        points_size = points_size_;

        sprt_histories[0]->A = estimateThresholdA(sprt_histories[0]->epsilon, sprt_histories[0]->delta);
        sprt_histories[0]->k = 0;

        number_rejected_models = 0;
        sum_fraction_data_points = 0;

        max_hypothesis_test_before_sprt = model->max_hypothesis_test_before_sprt;
    }

    /*
     *                      p(x(r)|Hb)                  p(x(j)|Hb)
     * lambda(j) = Product (----------) = lambda(j-1) * ----------
     *                      p(x(r)|Hg)                  p(x(j)|Hg)
     * Set j = 1
     * 1.  Check whether j-th data point is consistent with the
     * model
     * 2.  Compute the likelihood ratio λj eq. (1)
     * 3.  If λj >  A, decide the model is ’bad’ (model ”re-jected”),
     * else increment j or continue testing
     * 4.  If j = N the number of correspondences decide model ”accepted
     *
     * Verifies model and returns model score.
     */
    bool verifyModelAndGetModelScore (Model * model, int current_hypothese, unsigned int maximum_score, 
                                                                            Score *score) {

        estimator->setModelParameters(model->returnDescriptor());

        double epsilon = sprt_histories[current_sprt_idx]->epsilon;
        double delta = sprt_histories[current_sprt_idx]->delta;
        double A = sprt_histories[current_sprt_idx]->A;

//         std::cout << "epsilon = " << epsilon << "\n";
//         std::cout << "delta =  " << delta << "\n";
//         std::cout << "A =  " << A << "\n";

        double lambda_new, lambda = 1;

        unsigned int tested_point = 0, tested_inliers = 0;

        bool good = true;
        for (tested_point = 0; tested_point < points_size; tested_point++) {

            // reset pool index
            if (random_pool_idx >= points_size) {
                random_pool_idx = 0;
            }

//            std::cout << point << " ";
            if (estimator->GetError(points_random_pool[random_pool_idx]) < threshold) {
                tested_inliers++;
                lambda_new = lambda * (delta / epsilon);
            } else {
                lambda_new = lambda * ((1 - delta) / (1 - epsilon));
            }
            random_pool_idx++;

//              std::cout << "λ = " << lambda_new << " vs A = " << A << "\n";
            if (lambda_new > A) {
//                std::cout << "\n";
//                std::cout << "BAD MODEL IN " << tested_point << "/" << points_size << "\n";
                good = false;
                tested_point++;
                break;
            }
            lambda = lambda_new;
        }
//        std::cout << "\n";

        if (good) {
            // Assume that model is good or current iteration is less than threshold of tested hypothesis before applying sprt.
            // Otherwise we don't need model score, because model is bad.
            score->inlier_number = tested_inliers;
            score->score = score->inlier_number;
        } else
        if (current_hypothese < max_hypothesis_test_before_sprt) {
            // we still need the score in this case
            unsigned int inliers_after_test = 0;
            for (unsigned int p = tested_point; p < points_size; p++) {
                if (random_pool_idx >= points_size) {
                    random_pool_idx = 0;
                }
                if (estimator->GetError(points_random_pool[random_pool_idx]) < threshold) {
                    inliers_after_test++;
                }
                random_pool_idx++;
            }
            score->inlier_number = tested_inliers + inliers_after_test;
            score->score = score->inlier_number;
        }


        if (good) {
            /*
             * Model accepted and the largest support so far:
             * design (i+1)-th test (εi + 1= εˆ, δi+1 = δˆ, i = i + 1).
             * Store the current model parameters θ
             */
            if (tested_inliers > maximum_score) {
                auto * new_sprt_history = new SPRT_history;
//                 std::cout << "UPDATE. GOOD MODEL\n";

                new_sprt_history->epsilon = (float) tested_inliers / points_size;

                new_sprt_history->delta = delta;
                new_sprt_history->A = estimateThresholdA (new_sprt_history->epsilon, delta);

//                 new_sprt_history->delta = delta_estimated;
//                 new_sprt_history->A = estimateThresholdA (new_sprt_history->epsilon, delta_estimated);

                new_sprt_history->k = current_hypothese - last_sprt_update;
                last_sprt_update = current_hypothese;
                current_sprt_idx++;
                sprt_histories.push_back(new_sprt_history);
            }

        } else {
            /*
            * Since almost all tested models are ‘bad’, the probability
            * δ can be estimated as the average fraction of consistent data points
            * in rejected models.
            */
//             std::cout << "Reject model\n";
            float delta_estimated = (float) tested_inliers / tested_point;

//             number_rejected_models++;
//             sum_fraction_data_points += (float) tested_inliers / (float) tested_point;
//             float delta_estimated = (float) sum_fraction_data_points / (float) number_rejected_models;

//             std::cout << delta_estimated << " = delta est\n";
            if (delta_estimated > 0 && fabs(delta - delta_estimated) / delta > 0.05) {
                auto * new_sprt_history = new SPRT_history;
//                std::cout << "UPDATE. BAD MODEL\n";

                /*
                * Model rejected: re-estimate δ. If the estimate δ_ differs
                * from δi by more than 5% design (i+1)-th test (εi+1 = εi,
                * δi+1 = δˆ, i = i + 1)
                */
                new_sprt_history->epsilon = epsilon;
                new_sprt_history->delta = delta_estimated;
                new_sprt_history->A = estimateThresholdA(epsilon, delta_estimated);
                new_sprt_history->k = current_hypothese - last_sprt_update;
                last_sprt_update = current_hypothese;
                current_sprt_idx++;
                sprt_histories.push_back(new_sprt_history);
            }
        }
        return good;
    }


    /*
    * A(0) = K1/K2 + 1
    * A(n+1) = K1/K2 + 1 + log (A(n))
    * K1 = t_M / P_g
    * K2 = m_S/(P_g*C)
    * t_M is time needed to instantiate a model hypotheses given a sample
    * P_g = epsilon ^ m, m is the number of data point in the Ransac sample.
    * m_S is the number of models that are verified per sample.
    *                   p (0|Hb)                  p (1|Hb)
    * C = p(0|Hb) log (---------) + p(1|Hb) log (---------)
    *                   p (0|Hg)                  p (1|Hg)
    */
    double estimateThresholdA (double epsilon, double delta) {
        double C;
        C = (1 - delta) * log ((1 - delta)/(1-epsilon)) + delta * (log(delta/epsilon));
        // K = K1/K2 + 1 = (t_M / P_g) / (m_S / (C * P_g)) + 1 = (t_M * S)/m_S + 1
        double K = (t_M * C) / m_S + 1;
        double An_1 = K;
//         std::cout << "C = " << C << "\n";
//         std::cout << "K = " << K << "\n";
//         std::cout << An_1 << " An 1\n";
        // compute A using a recursive relation
        // A* = lim(n->inf)(An), the series typically converges within 4 iterations
        double An;
        for (unsigned int i = 0; i < 10; ++i) {
            An = K + log(An_1);

//             std::cout << An_1 << " An\n";
            if (fabs(An - An_1) < 1.5e-8) {
                break;
            }
            An_1 = An;
        }

        return An;
    }

    /*
     * Termination criteria
     * n(l) = Product from i = 0 to l ( 1 - Pg (1 - A(i)^(-h(i)))^k(i) )
     * log n(l) = sum from i = 0 to l k(i) * ( 1 - Pg (1 - A(i)^(-h(i))) )
     *
     *
     *        log (n0) - log (n(l-1))
     * k(l) = -----------------------
     *          log (1 - Pg*A(l)^-1)
     *
     * n0 is typically set to 0.05
     * this equation does not have to be evaluated before nR < n0
     * nR = (1 - P_g)^k
     */
    unsigned int getUpperBoundIterations (int inliers_size) {
        double epsilon = (double) inliers_size / points_size;
        double P_g = pow (epsilon, sample_size);
        double log_eta_l_1 = 0;
        double h;
        for (unsigned int test = 0; test < current_sprt_idx; test++) {
            h = computeExponentH(sprt_histories[test]->epsilon, epsilon, sprt_histories[test]->delta);
            log_eta_l_1 += log (1 - P_g * (1 - pow (sprt_histories[test]->A, -h))) * sprt_histories[test]->k;
        }
//         std::cout << n_l_1 << "\n";
        double numerator = LOG_ETA_0 - log_eta_l_1;
//         std::cout << numerator << " = numerator\n";
        if (numerator >= 0) return 0;
        double denumerator = log (1 - P_g * (1 - 1/sprt_histories[current_sprt_idx]->A));
//         std::cout << denumerator << " = denumerator\n";
        if (std::isnan(denumerator) || fabs (denumerator) < 0.00001)
            return max_iterations;

//         std::cout << log (1 - P_g/sprt_histories[current_sprt_idx]->A) << " down\n";
        double kl = numerator / denumerator;
//         std::cout << kl << " = kl\n";
        return (unsigned int) std::min ((unsigned int)kl , max_iterations);
    }


    // ------------ Usac version (Raguram, et.al) (not using, just for compare) ----------
    unsigned int updateSPRTStopping(unsigned int numInliers) {
        double n_inliers = 1.0;
        double n_pts = 1.0;
        double h = 0.0, k = 0.0, prob_reject_good_model = 0.0, log_eta = 0.0;
        double new_eps = (double)numInliers/points_size;

        for (unsigned int i = 0; i < sample_size; ++i) {
            n_inliers *= numInliers - i;
            n_pts *= points_size - i;
        }
        double prob_good_model = n_inliers/n_pts;

        if ( prob_good_model < std::numeric_limits<double>::epsilon() ) {
            return max_iterations;
        }
        else if ( 1 - prob_good_model < std::numeric_limits<double>::epsilon() )
        {
            return 1;
        }

        for (unsigned int test = 0; test < current_sprt_idx; test++) {
            k += sprt_histories[test]->k;
            h = computeExponentH(new_eps, sprt_histories[test]->epsilon, sprt_histories[test]->delta);
            prob_reject_good_model = 1/(exp( h*log(sprt_histories[test]->A) ));
            log_eta += (double) sprt_histories[test]->k * log( 1 - prob_good_model*(1-prob_reject_good_model) );
        }

        double nusample_s = k + ( log(1-confidence) - log_eta ) / log( 1-prob_good_model * (1-(1/sprt_histories[current_sprt_idx]->A)) );
        return (unsigned int) ceil(nusample_s);
    }
    //-----------------------------

    /*
     * h(i) must hold
     *
     *     δ(i)                  1 - δ(i)
     * ε (-----)^h(i) + (1 - ε) (-------)^h(i) = 1
     *     ε(i)                  1 - ε
     *
     * ε * a^h + (1 - ε) * b^h = 1
     * f (h) = ε * a^h + (1 - ε) * b^h - 1, h = ? => f(h) = 0 (roots searching)
     * f'(h) = ε * a^h + (1 - ε) * b^h
     * Leads to numerical solution (bisection, newton, taylor, ...)
     * Newton: h(k+1) = h(k) - f(h(k))/f'(h(k))
     */
    double computeExponentH (double epsilon, double epsilon_new, double delta) {

        double a, b, x0, x1, v0, v1;
        a = log(delta/epsilon);
        b = log ((1-delta)/(1-epsilon));

        x0 = log (1/(1-epsilon_new))/b;
        v0 = epsilon_new * exp(x0 * a);
        x1 = log ((1-2*v0) / (1-epsilon_new))/b;
        v1 = epsilon_new * exp(x1 * a) + (1 - epsilon_new) * exp(x1 * b);
        double h = x0 - (x0 - x1)/(1+v0 - v1)*v0;

        // OR:
//         a = delta / epsilon;
//         b = (1 - delta) / (1 - epsilon);
//         auto fh = [&] (double h) { return epsilon_new * pow (a, h) + (1 - epsilon_new) * pow (b, h) - 1; };
//         auto dfh = [&] (double h) { return epsilon_new * pow (a, h) + (1 - epsilon_new) * pow (b, h); };
//         unsigned int iter = 0;
//         double hn = 0, hn_1 = 2;
//         while (abs (fh (hn_1)) > 0.0001 || iter < 100) {
//             hn = hn_1 - fh (hn_1) / dfh(hn_1);
//             hn_1 = hn;
//             iter++;
//         }

//         std::cout << "h = " << h << "\n";
//         std::cout << "hn = " << hn << "\n";

        if (std::isnan(h)) {
            // The equation always has solution for h = 0
            // ε * a^0 + (1 - ε) * b^0 = 1
            // ε + 1 - ε = 1 -> 1 = 1
            return 0;
        }

        // std::cout << "------compute h--------\n";
//          std::cout << "epsilon " << epsilon << '\n';
//          std::cout << "epsilon_new " << epsilon_new << '\n';
//          std::cout << "delta " << delta << '\n';
//          std::cout << "al " << al << '\n';
//            std::cout << "h = " << (x0 - (x0 - x1)/(1+v0 - v1)*v0) << "\n";
        // std::cout << "be " << be << '\n';
        // std::cout << "v0 " << v0 << '\n';
        // std::cout << "v1 " << v1 << '\n';
        // std::cout << "x0 " << x0 << '\n';
        // std::cout << "x1 " << x1 << '\n';
        // std::cout << "------------------------------------------\n";

        return h;
    }
};

#endif // SPRT_H