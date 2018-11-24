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

#include "Model.h"
#include "Utils/Math.h"
#include "Estimator/Estimator.h"

/*
 * Sequential Probability Ratio Test
 *
 * From:
 * Randomized RANSAC with sequential probability ratio test
 * Authors: J. Matas, O. Chum
 * https://ieeexplore.ieee.org/document/1544925
 */

class SPRT_history {
public:
    float epsilon, delta, A;
    int k;
};

 class SPRT {
 private:
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
     int current_sprt_idx;
     int last_sprt_update;

     float t_M, m_S, threshold, prob_threshold;
     bool is_init = false;
     int points_size, sample_size, max_iterations;
     std::vector<SPRT_history*> sprt_histories;
     float number_points_product;
 public:

     ~SPRT() {
         sprt_histories.clear();
     }

     void initialize (Model * model, int points_size_) {
         sprt_histories = std::vector<SPRT_history*>();
        sprt_histories.push_back(new SPRT_history);

        if (model->estimator == ESTIMATOR::Homography) {
            // t_M = 200, m_S = 1, delta0 = 0.01, epsilon0 = 0.1;
            sprt_histories[0]->delta = 0.01;
            sprt_histories[0]->epsilon = 0.1;
            t_M = 200;
            m_S = 1;
            // K1 = 200 / 0.1^4 = 2 000 000
            // C = (1 - delta) * log ((1 - delta)/(1 - eps)) + delta * log (delta / eps)
            // C = 0.99 * log (0.99 / 0.9) + 0.01 * log (0.01 / 0.1) = 0.0713
            // K2 = 1 / (0.1^4 * 0.0713) = 140252.45
            // A = K1 + K2
        } else if (model->estimator == ESTIMATOR::Fundamental) {
            // t_M = 200, m_S = 2.48, delta0 = 0.05, epsilon0 = 0.2;
            sprt_histories[0]->delta = 0.05;
            sprt_histories[0]->epsilon = 0.2;
            t_M = 200;
            m_S = 2.48;
        } else {
            t_M = 200;
            m_S = 1;
            /*
             * The initial estimate δ0 is obtained by geometric considerations,
             * i.e. as a fraction of the area that supports a hypothesised model
             * (a strip around an epipolar line in case of epipolar geometry)
             * to the area of possible appearance of outlier data (the area of
             * the search window). Alternatively, a few models can be evaluated
             * without applying SPRT in order to obtain an initial estimate of δ.
             */
            sprt_histories[0]->delta = 0.01; // ???

            /*
             * The initial value of ε0 can be derived from the maximal time the
             * user is willing to allocate to the algorithm.
             */
            sprt_histories[0]->epsilon = 0.1; // ???

        }

         current_sprt_idx = 0;
         last_sprt_update = 0;

         sample_size = model->sample_number;
         threshold = model->threshold;
         prob_threshold = model->desired_prob;
         max_iterations = model->max_iterations;
         points_size = points_size_;

         number_points_product = 1;
         for (int i = 0; i < sample_size; i++) {
             number_points_product *= points_size - i;
         }

         sprt_histories[0]->A = estimateThresholdA(sprt_histories[0]->epsilon, sprt_histories[0]->delta);
         sprt_histories[0]->k = 0;

         is_init = true;
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
     void verifyModelAndGetModelScore (Estimator * estimator, Model * model, int current_hypothese, 
                    int maximum_score, bool get_score, Score *score, bool get_inliers, int * inliers) {
         
         estimator->setModelParameters(model->returnDescriptor());

         float epsilon = sprt_histories[current_sprt_idx]->epsilon;
         float delta = sprt_histories[current_sprt_idx]->delta;
         float A = sprt_histories[current_sprt_idx]->A;
         float lambda_new, lambda = 1;        

         int tested_inliers = 0;
         int tested_point = 0;
         
         bool good = true;
         for (tested_point = 0; tested_point < points_size; tested_point++) {
             if (estimator->GetError(tested_point) < threshold) {
                 if (get_inliers) {
                    inliers[tested_inliers] = tested_point;
                 }
                 tested_inliers++;
                 lambda_new = lambda * (delta / epsilon);
             } else {
                 lambda_new = lambda * ((1 - delta) / (1 - epsilon));
             }

             if (lambda_new < A) {
                 good = false;
                 tested_point++;
                 break;
             }
             lambda = lambda_new;   
         }
         
         if (get_score) {
            int total_inliers = tested_inliers; 
            if (get_inliers) {
                for (int j = tested_point+1; j < points_size; j++) {
                    if (estimator->GetError (j) < threshold) {
                        inliers[total_inliers++] = j;
                    }
                }
            } else {
                for (int j = tested_point+1; j < points_size; j++) {
                    if (estimator->GetError (j) < threshold) {
                        total_inliers++;
                    }
                }
            }
            score->inlier_number = total_inliers;
            score->score = total_inliers;
         }

          /*
           * Since almost all tested models are ‘bad’, the probability
           * δ can be estimated as the average fraction of consistent data points
           * in rejected models.
           * ???????????????????
           */
         float delta_estimated = (float) tested_inliers / tested_point;

         if (good) {
             /*
              * Model accepted and the largest support so far:
              * design (i+1)-th test (εi + 1= εˆ, δi+1 = δˆ, i = i + 1).
              * Store the current model parameters θ
              */
             if (tested_inliers > maximum_score) {
                 SPRT_history * new_sprt_history = new SPRT_history;

                 new_sprt_history->epsilon = (float) tested_inliers / points_size;
                 new_sprt_history->delta = delta_estimated;
                 new_sprt_history->A = estimateThresholdA (new_sprt_history->epsilon, delta_estimated);
                 new_sprt_history->k = current_hypothese - last_sprt_update;
                 last_sprt_update = current_hypothese;
                 current_sprt_idx++;
                 sprt_histories.push_back(new_sprt_history);
             }

         } else {
             if (delta_estimated > 0 && fabsf(delta - delta_estimated) / delta > 0.1) {
                 SPRT_history * new_sprt_history = new SPRT_history;

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
     float estimateThresholdA (float epsilon, float delta) {

         float C = (1 - delta) * log ((1 - delta)/(1-epsilon)) + delta * (log(delta/epsilon));
         // K = K1/K2 + 1 = (t_M / P_g) / (m_S / (C * P_g)) + 1= (t_M * S)/m_S + 1
         float K = (t_M * C)/m_S + 1;
         float An_1 = K;

         // compute A using a recursive relation
         // A* = lim(n->inf)(An), the series typically converges within 4 iterations
         float An;
         for (unsigned int i = 0; i < 10; ++i) {
             An = K + log(An_1);
             if (An - An_1 < 1.5e-8) {
                 break;
             }
             An_1 = An;
         }

         return An;
     }

     /*
      * Termination criteria
      * n(l) = Product from i = 0 to l ( 1 - Pg (1 - A(i)^(-h(i)))^k(i) )
      *
      *        log (n0) - log (n(l-1))
      * k(l) = -----------------------
      *          log (1 - Pg*A(l)^-1)
      */
     int getMaximumIterations (int inliers_size) {
         // debug
         // for (int t = 0; t <= current_sprt_idx; t++) {
         //     std::cout << "test " << t << "\n";
         //     std::cout << "e = " <<  sprt_histories[t]->epsilon << "\n";
         //     std::cout << "d = " << sprt_histories[t]->delta << "\n";
         //     std::cout << "A = " << sprt_histories[t]->A << "\n";
         //     std::cout << "----\n";
         // }


         double n_inliers = 1.0;
         double n_pts = 1.0;
         double h, k = 0, prob_reject_good_model, log_eta = 0;
         double new_eps = (double) inliers_size / points_size;

         for (unsigned int i = 0; i < sample_size; ++i) {
             n_inliers *= inliers_size - i;
             n_pts *= points_size - i;
         }

         double prob_good_model = n_inliers / n_pts;
         // std::cout << "prob_good_model " << prob_good_model << '\n';

         if (prob_good_model < std::numeric_limits<double>::epsilon() ) {
             return max_iterations;
         } else if (1 - prob_good_model < std::numeric_limits<double>::epsilon() ) {
             return 1;
         }

         for (int test = 0; test < current_sprt_idx; test++) {
             k += sprt_histories[test]->k;
             h = computeExponentH(sprt_histories[test]->epsilon, new_eps, sprt_histories[test]->delta);
             prob_reject_good_model = 1/(exp( h*log(sprt_histories[test]->A) ));

             // std::cout << "k = " << k << '\n';
             // std::cout << "h = " << h << '\n';
             // std::cout << "prob reject good model " << prob_reject_good_model << "\n";

             log_eta += sprt_histories[test]->k * log( 1 - prob_good_model*(1-prob_reject_good_model) );
         }

         double nusample_s = k + ( log(1-prob_threshold) - log_eta ) / log (1-prob_good_model * (1-(1/sprt_histories[current_sprt_idx]->A)) );
         return nusample_s > max_iterations ? max_iterations : (unsigned int) ceil(nusample_s);
     }

     /*
      * h(i) must hold
      *
      *     δ(i)                  1 - δ(i)
      * ε (-----)^h(i) + (1 - ε) (-------)^h(i) = 1
      *     ε(i)                  1 - ε
      *
      */
     float computeExponentH (float epsilon, float epsilon_new, float delta) {

         float al, be, x0, x1, v0, v1;

         al = log(delta/epsilon);
         be = log( (1-delta)/(1-epsilon) );

         x0 = log( 1/(1-epsilon_new) )/be;
         v0 = epsilon_new * exp(x0 *al);
         x1 = log( (1-2*v0) / (1-epsilon_new) )/be;
         v1 = epsilon_new * exp(x1 * al) + (1-epsilon_new) * exp(x1 * be);

         // std::cout << "------compute h--------\n";
         // std::cout << "epsilon " << epsilon << '\n';
         // std::cout << "epsilon_new " << epsilon_new << '\n';
         // std::cout << "delta " << delta << '\n';
         // std::cout << "al " << al << '\n';
         // std::cout << "be " << be << '\n';
         // std::cout << "v0 " << v0 << '\n';
         // std::cout << "v1 " << v1 << '\n';
         // std::cout << "x0 " << x0 << '\n';
         // std::cout << "x1 " << x1 << '\n';
         // std::cout << "------------------------------------------\n";

         return x0 - (x0 - x1)/(1+v0 - v1)*v0; // h
     }


     bool isInit () {
         return is_init;
     }
 };

#endif // SPRT_H