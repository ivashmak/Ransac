#ifndef SPRT_H
#define SPRT_H

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

 class SPRT {
 private:
     /*
      * The probability of a data point being consistent
      * with a ‘bad’ model is modeled as a probability of
      * a random event with Bernoulli distribution with parameter
      * δ : p(1|Hb) = δ.
      */
     float delta_i;
     float delta_i_new;

    /*
     * The probability p(1|Hg) = ε
     * that any randomly chosen data point is consistent with a ‘good’ model
     * is approximated by the fraction of inliers ε among the data
     * points
     */
     float epsilon_i;
     float epsilon_i_new;

     /*
      * The decision threshold A is the only parameter of the Adapted SPRT
      */
     float Ai;
     int i;
     float t_M, m_S, threshold;
     bool is_init = false;
     int points_size, sample_size;
 public:

     void initialize (Model * model) {
        if (model->estimator == ESTIMATOR::Homography) {
            // t_M = 200, m_S = 1, delta0 = 0.01, epsilon0 = 0.1;
            delta_i = 0.01;
            epsilon_i = 0.1;
            t_M = 200;
            m_S = 1;
            // K1 = 200 / 0.1^4 = 2 000 000
            // C = (1 - delta) * log ((1 - delta)/(1 - eps)) + delta * log (delta / eps)
            // C = 0.99 * log (0.99 / 0.9) + 0.01 * log (0.01 / 0.1) = 0.0713
            // K2 = 1 / (0.1^4 * 0.0713) = 140252.45
            // A = K1 + K2
        } else if (model->estimator == ESTIMATOR::Fundamental) {
            // t_M = 200, m_S = 2.48, delta0 = 0.05, epsilon0 = 0.2;
            delta_i = 0.05;
            epsilon_i = 0.2;
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
            delta_i = 0.01; // ???

            /*
             * The initial value of ε0 can be derived from the maximal time the
             * user is willing to allocate to the algorithm.
             */
            // model->max_iters;
            epsilon_i = 0.1; // ???

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
        }

         i = 0;
         sample_size = model->sample_number;
         threshold = model->threshold;
         epsilon_i_new = epsilon_i;
         delta_i_new = delta_i;

         Ai = 1; // log (Ai) = log (1) = 0
         Ai = estimateA();

         is_init = true;
     }

     /*
      * eq (1)
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
      */
     bool isGoodModel (Estimator * estimator, Model * model) {
         estimator->setModelParameters(model);

         float lambda = 1;
         float lambda_new;
         bool good = true;
         float error;
         for (int j = 0; j < points_size; j++) {
             error = estimator->GetError(j);

             if (error < threshold) {
                 // x(j) is inlier
                 lambda_new = lambda * (delta_i_new / epsilon_i_new);
             } else {
                 lambda_new = lambda * ((1 - delta_i_new) / (1 - epsilon_i_new));
             }

             if (lambda_new < Ai) {
                 good = false;
                 break;
             }
         }

         float delta_i_new_temp = estimateDelta();
         if (good) {
             /*
              * Model accepted and the largest support so far:
              * design (i+1)-th test (εi + 1= εˆ, δi+1 = δˆ, i = i + 1).
              * Store the current model parameters θ
              */
             i = i + 1;
             epsilon_i_new = estimateEpsilon(0);
             delta_i_new = delta_i_new_temp;
         } else if ((delta_i_new_temp - delta_i) > 0.05 * delta_i) {
             /*
              * Model rejected: re-estimate δ. If the estimate δ_ differs
              * from δi by more than 5% design (i+1)-th test (εi+1 = εi,
              * δi+1 = δ_, i = i+ 1)
              */
             i = i + 1;
             epsilon_i_new = epsilon_i;
             delta_i_new = delta_i_new_temp;
         } else {
             delta_i_new = delta_i;
         }

         Ai = estimateA();
         return good;
     }

     /*
      * Since almost all tested models are ‘bad’, the probability
      * δ can be estimated as the average fraction of consistent data points
      * in rejected models.
      */
     float estimateDelta () {
         return 0;
     }

     /*
      * epsilon (i+1) = I(i+1) / N;
      */
     float estimateEpsilon (int inliers_size) {
         return inliers_size / points_size;
     }

     float estimateA () {
         float C, K1, K2, P_g;
         P_g = fast_pow (epsilon_i_new, sample_size);
         C = (1 - delta_i_new) * log ((1-delta_i_new)/(1-epsilon_i_new)) + delta_i_new * log (delta_i_new / epsilon_i_new);
         K1 = t_M / P_g;
         K2 = m_S / (C * P_g);
         return K1 / K2 + 1 + log(Ai);
     }

     /*
      * Termination criteria
      * n(l) = Product from i = 0 to l ( 1 - Pg (1 - A(i)^(-h(i)))^k(i) )
      */
     int getMaximumIterations  () {

     }

     bool isInit () {
         return is_init;
     }
 };

#endif // SPRT_H