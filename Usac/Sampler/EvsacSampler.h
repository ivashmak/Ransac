#ifndef USAC_EVSACSAMPLER_H
#define USAC_EVSACSAMPLER_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>

#include "Sampler.h"
#include <theia/theia.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

struct MixtureModelParams {
    // Gamma Parameters (for correspondences modeled to be correct).
    double k;      // shape.
    double theta;  // scale.

    // GEV Parameters (for correspondences estimated to be incorrect).
    double xi;     // tail.
    double sigma;  // scale.
    double mu;     // location.

    // Inlier ratio (Mixture parameter).
    double inlier_ratio;  // Estimated inlier ratio.
};

typedef optimo::solvers::PrimalDualQP<double,
        2 /* Num. Unknowns */,
        4 /* Num. Inequalities */,
        1 /* Num. Equalities */>
        PrimalDualQP;


//The matrix containing k L2 sorted distances in ascending order. The matrix has
//        num. of query features as rows and k columns.

class EvsacSampler : public Sampler {
protected:
    std::unique_ptr<std::discrete_distribution<>> correspondence_sampler_;
    std::mt19937 rng_;
    MixtureModelParams mixture_model_params;
    theia::FittingMethod fitting_method;
    double predictor_threshold;
public:

    EvsacSampler (cv::InputArray input_points1, cv::InputArray input_points2, int knn) {
        // init random generator
        srand (time(NULL));
        std::random_device rand_dev;
        rng_.seed(rand_dev());


        // get initial point
        int total_points = input_points1.rows();
        std::cout << "total_points = " << total_points << '\n';
        cv::Mat points1 = cv::Mat(total_points, 2, CV_32F, input_points1.getMat().data);
        cv::Mat points2 = cv::Mat(total_points, 2, CV_32F, input_points2.getMat().data);
        int * init_point = new int[1];
        init_point[0] = rand() % total_points;
        cv::Point_<float> initial_point = points1.at<cv::Point_<float>>(init_point[0]);
        cv::Point_<float> initial_point_corr = points2.at<cv::Point_<float>>(init_point[0]);


        //  The idea of EVSAC is to model the statistics of the minimum distances
        // computed when using the Nearest-Neighbor feature matcher.
        // find knn and their sorted distances.
        cv::Mat query, sorted_distances1, sorted_distances2, indicies;
        cv::flann::LinearIndexParams flannIndexParams;
        cv::flann::Index flannIndex1 (cv::Mat(points1).reshape(1), flannIndexParams);
        cv::flann::Index flannIndex2 (cv::Mat(points2).reshape(1), flannIndexParams);
        query = cv::Mat(1, 2, CV_32F, &initial_point);
        flannIndex1.knnSearch(query, indicies, sorted_distances1, knn);
        query = cv::Mat(1, 2, CV_32F, &initial_point_corr);
        flannIndex2.knnSearch(query, indicies, sorted_distances2, knn);

        std::cout << sorted_distances1 << "\n\n";
        std::cout << sorted_distances2 << "\n\n";


        // fitting_method:  The fitting method MLE or QUANTILE_NLS (see statx doc).
        //   The recommended fitting method is the MLE estimation.
        this->fitting_method = theia::QUANTILE_NLS;


        // The threshold used to decide correct or incorrect
        // matches/correspondences. The recommended value is 0.65.
        this->predictor_threshold = 0.65;


        std::vector<float> probabilities;
        std::vector<float> sampling_weights;

        Eigen::MatrixXd sorted_distances_(knn, 2);
        for (int i = 0; i < knn; i++) {
            sorted_distances_(i, 0) = sorted_distances1.at<float>(i);
            sorted_distances_(i, 1) = sorted_distances2.at<float>(i);
        }

        // mixture samples: matrix of 50 x 6 containing in the first column samples of a
        // gamma(3, 2) and the 2:6 columns are sorted samples taken from a chi2rnd(128).
        // This is a separable case. The data is written using a column major.
        // This data was generated with MATLAB.
        const std::vector<double> mixture_samples = {
                8.453391e+00, 6.643481e+00, 7.073280e+00, 7.620230e-01, 4.796192e+00,
                4.265579e+00, 7.504951e+00, 6.491244e+00, 1.086329e+01, 1.008544e+01,
                4.580231e+00, 2.009185e+00, 4.833628e+00, 2.524501e+00, 5.473356e+00,
                1.412114e+01, 7.078814e+00, 5.815086e+00, 5.774830e+00, 7.891432e+00,
                4.030500e+00, 3.868219e+00, 7.489940e+00, 1.655775e+00, 2.400819e+00,
                3.553688e+00, 1.342447e+01, 5.690786e+00, 1.377225e+01, 2.944703e+00,
                6.516840e+00, 4.445703e+00, 8.211898e+00, 3.078726e+00, 1.199771e+01,
                4.297356e+00, 3.419880e+00, 3.356973e+00, 1.402518e+01, 6.971956e+00,
                4.778875e+00, 5.016027e+00, 1.416609e+00, 4.515507e+00, 6.002494e+00,
                1.032861e+01, 5.377348e+00, 5.531420e+00, 5.714113e+00, 2.514066e+00,
                7.747292e+01, 8.828736e+01, 7.707717e+01, 6.252057e+01, 8.828689e+01,
                7.832088e+01, 8.107225e+01, 8.690371e+01, 8.368767e+01, 8.400832e+01,
                7.684186e+01, 7.281568e+01, 8.273788e+01, 8.661383e+01, 7.883183e+01,
                8.208945e+01, 7.917460e+01, 7.973665e+01, 8.819710e+01, 8.426296e+01,
                8.509478e+01, 8.832398e+01, 8.264218e+01, 8.444737e+01, 7.573799e+01,
                8.841938e+01, 7.811256e+01, 7.576132e+01, 8.681609e+01, 8.489867e+01,
                8.107643e+01, 8.662812e+01, 8.254524e+01, 7.886447e+01, 8.168945e+01,
                7.817184e+01, 8.283279e+01, 8.639165e+01, 7.494652e+01, 9.019843e+01,
                8.972476e+01, 8.043246e+01, 7.397081e+01, 8.708566e+01, 8.882751e+01,
                8.366903e+01, 7.833746e+01, 8.907670e+01, 8.344260e+01, 8.655409e+01,
                8.729785e+01, 8.922214e+01, 7.845613e+01, 7.889089e+01, 9.060706e+01,
                8.381995e+01, 8.526234e+01, 8.729997e+01, 8.701973e+01, 8.490448e+01,
                8.105177e+01, 8.261784e+01, 8.548979e+01, 8.890356e+01, 8.402725e+01,
                8.315342e+01, 8.384340e+01, 8.553590e+01, 8.991483e+01, 8.788811e+01,
                8.534967e+01, 8.863323e+01, 8.544508e+01, 8.514051e+01, 8.376380e+01,
                8.991751e+01, 8.346599e+01, 8.031017e+01, 9.317175e+01, 8.753779e+01,
                8.496033e+01, 8.801973e+01, 8.618550e+01, 7.970668e+01, 8.680145e+01,
                8.072150e+01, 8.624960e+01, 8.651754e+01, 8.669568e+01, 9.083972e+01,
                8.974145e+01, 8.887377e+01, 8.312898e+01, 9.055060e+01, 8.900577e+01,
                8.639638e+01, 7.949458e+01, 9.038057e+01, 8.498672e+01, 8.846037e+01,
                8.816720e+01, 9.004428e+01, 8.555926e+01, 8.459183e+01, 9.204340e+01,
                8.459763e+01, 8.829754e+01, 8.779764e+01, 8.741255e+01, 8.579928e+01,
                8.524173e+01, 9.019785e+01, 8.968418e+01, 9.033841e+01, 8.494151e+01,
                8.476563e+01, 8.703247e+01, 8.642406e+01, 9.083866e+01, 8.802507e+01,
                8.853094e+01, 9.004607e+01, 8.565775e+01, 8.636985e+01, 8.489890e+01,
                9.017141e+01, 8.671391e+01, 8.102536e+01, 9.395337e+01, 8.863653e+01,
                8.718898e+01, 9.115773e+01, 8.737245e+01, 8.339203e+01, 8.775825e+01,
                8.745352e+01, 8.636032e+01, 8.897125e+01, 8.724385e+01, 9.114361e+01,
                9.073200e+01, 8.902639e+01, 8.513250e+01, 9.055553e+01, 9.092100e+01,
                8.699845e+01, 8.183630e+01, 9.096764e+01, 8.619657e+01, 8.918183e+01,
                8.895334e+01, 9.072155e+01, 8.596572e+01, 8.686888e+01, 9.209827e+01,
                8.697255e+01, 8.974051e+01, 8.995643e+01, 9.067128e+01, 8.862904e+01,
                8.845180e+01, 9.219812e+01, 9.056826e+01, 9.044349e+01, 8.776265e+01,
                8.749650e+01, 8.801521e+01, 8.643755e+01, 9.125138e+01, 9.063493e+01,
                8.882734e+01, 9.186411e+01, 8.697559e+01, 8.640061e+01, 8.496423e+01,
                9.220204e+01, 8.738610e+01, 8.893682e+01, 9.421022e+01, 9.167722e+01,
                8.816749e+01, 9.155549e+01, 8.850337e+01, 8.341849e+01, 8.962811e+01,
                9.011022e+01, 8.938827e+01, 8.948222e+01, 8.724831e+01, 9.150108e+01,
                9.092604e+01, 8.930900e+01, 8.946705e+01, 9.060121e+01, 9.120959e+01,
                8.773249e+01, 8.563068e+01, 9.159597e+01, 8.808799e+01, 8.918286e+01,
                8.926355e+01, 9.278073e+01, 9.017833e+01, 8.710429e+01, 9.319228e+01,
                8.887087e+01, 8.999791e+01, 9.042229e+01, 9.158277e+01, 9.135374e+01,
                8.878243e+01, 9.259866e+01, 9.157813e+01, 9.142548e+01, 8.943605e+01,
                8.861633e+01, 8.915538e+01, 8.646916e+01, 9.144094e+01, 9.096172e+01,
                8.895131e+01, 9.328853e+01, 8.875806e+01, 8.644363e+01, 8.676286e+01,
                9.310095e+01, 8.763685e+01, 8.922829e+01, 9.451073e+01, 9.261248e+01,
                8.873643e+01, 9.319027e+01, 8.863945e+01, 8.480577e+01, 9.076276e+01,
                9.062156e+01, 8.950944e+01, 9.012737e+01, 9.051290e+01, 9.264635e+01,
                9.108225e+01, 9.045001e+01, 9.012979e+01, 9.157871e+01, 9.429332e+01,
                8.795334e+01, 8.852589e+01, 9.203239e+01, 8.977151e+01, 8.969222e+01
        };
        const Eigen::MatrixXd distances =
                Eigen::Map<const Eigen::MatrixXd>(&mixture_samples[0], 50, 6);

//        std::cout << distances << "\n\n";
        std::cout << sorted_distances_ << "\n\n";

//        theia::EvsacSampler<Eigen::Vector2d> ev_s;
//        int r = theia::EvsacSampler<Eigen::Vector2d>::CalculateKSmallestDistances(100, 0.01);

        bool r2 = theia::EvsacSampler<Eigen::Vector2d>::CalculateMixtureModel(sorted_distances_,
                                                                              this->predictor_threshold,
                                                                              this->fitting_method,
                                                                              reinterpret_cast<theia::EvsacSampler<Eigen::Vector2d>::MixtureModelParams *>(&this->mixture_model_params),
                                                                              &probabilities,
                                                                              &sampling_weights);

        std::cout << r2 << '\n';
        exit(0);

        bool res = CalculateMixtureModel(sorted_distances_,
                               this->predictor_threshold,
                               this->fitting_method,
                               &this->mixture_model_params,
                               &probabilities,
                               &sampling_weights);
        if (!res) {
            std::cout << "CalculateMixtureModel failed\n";
            exit(0);
        }

        std::cout << "initializeSampler\n";
        initializeSampler(sampling_weights);
    }

    bool CalculateMixtureModel(const Eigen::MatrixXd sorted_distances,
                               const double predictor_threshold,
                               const theia::FittingMethod fitting_method,
                               MixtureModelParams *mixture_model_params,
                               std::vector<float> *probabilities,
                               std::vector<float> *sampling_weights) {

        // A container indicating if a query point is predicted as correct or
        // incorrect.
        std::vector<bool> predicted_correct_correspondences;
        // The minimum distances, i.e., the first column of sorted_distances.
        std::vector<double> smallest_distances;
        // The second column of sorted_distances.
        std::vector<double> negated_second_smallest_distances;
        // Distances of correspondences that were predicted as correct, i.e., inliers.
        std::vector<double> predicted_inlier_distances;

        const double inlier_ratio_upper_bound =
                ExtractDataForFittingDistributions(sorted_distances,
                                                   predictor_threshold,
                                                   &predicted_correct_correspondences,
                                                   &smallest_distances,
                                                   &negated_second_smallest_distances,
                                                   &predicted_inlier_distances);

        // Fit gamma distribution.
        if (!FitGamma(predicted_inlier_distances, mixture_model_params)) {
            std::cout << "FitGamma failed.\n";
            return false;
        }

        // Fit GEV distribution.
        if (!FitGEV(fitting_method,
                    negated_second_smallest_distances,
                    mixture_model_params)) {
            std::cout << "FitGEV failed.\n";
            return false;
        }

        // Estimate inlier ratio.
        if (!EstimateInlierRatio(
                inlier_ratio_upper_bound, smallest_distances, mixture_model_params)) {
            std::cout << "EstimateInlierRatio failed.\n";
            return false;
        }

        // Calculate posterior and final weights.
        ComputePosteriorAndWeights(sorted_distances.rows(),
                                   *mixture_model_params,
                                   smallest_distances,
                                   predicted_correct_correspondences,
                                   probabilities,
                                   sampling_weights);

        return true;
    }

    void ComputePosteriorAndWeights(
            const int num_correspondences,
            const MixtureModelParams& mixture_model_params,
            const std::vector<double>& smallest_distances,
            const std::vector<bool>& predictions,
            std::vector<float>* probabilities,
            std::vector<float>* sampling_weights) {
        CHECK_NOTNULL(probabilities)->resize(num_correspondences);
        CHECK_NOTNULL(sampling_weights)->resize(num_correspondences);
        for (int i = 0; i < num_correspondences; i++) {
            // Calculate posterior.
            const double gam_val =
                    mixture_model_params.inlier_ratio *
                    statx::distributions::gammapdf(smallest_distances[i],
                                                   mixture_model_params.k,
                                                   mixture_model_params.theta);
            const double gev_val =
                    (1.0 - mixture_model_params.inlier_ratio) *
                    statx::distributions::evd::gevpdf(smallest_distances[i],
                                                      mixture_model_params.mu,
                                                      mixture_model_params.sigma,
                                                      mixture_model_params.xi);
            const double posterior = gam_val / (gam_val + gev_val);
            // Removing those matches that are likely to be incorrect.
            (*probabilities)[i] = static_cast<float>(posterior);
            (*sampling_weights)[i] =
                    predictions[i] ? static_cast<float>(posterior) : 0.0f;
        }
    }

    double ExtractDataForFittingDistributions(
            const Eigen::MatrixXd& sorted_distances,
            const double predictor_threshold,
            std::vector<bool>* predicted_correct_correspondences,
            std::vector<double>* smallest_distances,
            std::vector<double>* negated_second_smallest_distances,
            std::vector<double>* predicted_correct_correspondences_distances) {

        CHECK_NOTNULL(predicted_correct_correspondences_distances)
                ->reserve(sorted_distances.rows());
        CHECK_NOTNULL(predicted_correct_correspondences)
                ->resize(sorted_distances.rows());
        CHECK_NOTNULL(smallest_distances)->resize(sorted_distances.rows());
        CHECK_NOTNULL(negated_second_smallest_distances)
                ->resize(sorted_distances.rows());

        double estimated_inlier_ratio = 0.0;
        for (int i = 0; i < sorted_distances.rows(); ++i) {
            // Copying the first column from sorted distances to estimate the parameters
            // of the gamma distribution.
            (*smallest_distances)[i] = sorted_distances(i, 0);
            // Copying the second column from sorted distances as negated numbers to
            // estimate the parameters of the reversed GEV distribution.
            (*negated_second_smallest_distances)[i] = -sorted_distances(i, 1);
            // Saving the predictions.
            (*predicted_correct_correspondences)[i] =
                    MRRayleigh(sorted_distances.row(i), predictor_threshold);
            if ((*predicted_correct_correspondences)[i]) {
                predicted_correct_correspondences_distances->push_back(
                        sorted_distances(i, 0));
                estimated_inlier_ratio += 1.0;
            }
        }

        return estimated_inlier_ratio / sorted_distances.rows();
    }


    bool MRRayleigh(const Eigen::RowVectorXd& sorted_distances,
                    const double predictor_threshold) {
        CHECK_GT(predictor_threshold, 0.0);
        CHECK_LE(predictor_threshold, 1.0);
        const std::vector<double> tail(
                sorted_distances.data() + 1,
                sorted_distances.data() + sorted_distances.size());
        // Fit distribution tail!
        const double sigma = statx::distributions::raylfit(tail);
        // Calculate belief of correctness.
        const double confidence =
                1.0 - statx::distributions::raylcdf(sorted_distances[0], sigma);
        return confidence >= predictor_threshold;
    }

    // Initialize correspondence sampler.
    void initializeSampler (std::vector<float> sampling_weights) {
        // MSVC has one of the constructors missing. See
        // http://stackoverflow.com/questions/21959404/initialising-stddiscrete-distribution-in-vs2013

        #if defined(_MSC_VER) & _MSC_VER < 1900
            std::size_t i(0);
            correspondence_sampler_.reset(new std::discrete_distribution<int>(sampling_weights.size(),
                                          0.0,  // dummy!
                                          0.0,  // dummy!
                                          [&sampling_weights, &i](double) {
                                            auto w = sampling_weights[i];
                                            ++i;
                                            return w;
                                          }));
        #else
            correspondence_sampler_.reset(new std::discrete_distribution<int>(
                sampling_weights.begin(), sampling_weights.end()));
        #endif
    }



    bool FitGamma(
            const std::vector<double>& predicted_correct_correspondences_distances,
            MixtureModelParams* mixture_model_parameters) {
        const bool gam_success = statx::distributions::gammafit(
                predicted_correct_correspondences_distances,
                &CHECK_NOTNULL(mixture_model_parameters)->k,
                &mixture_model_parameters->theta);
        std::cout << "Gamma distribution: k=" << mixture_model_parameters->k
                << " theta=" << mixture_model_parameters->theta
                << " flag: " << gam_success << '\n';
        return gam_success;
    }


    bool FitGEV(
            const theia::FittingMethod fitting_method,
            const std::vector<double>& negated_second_smallest_distances,
            MixtureModelParams* mixture_model_parameters) {
        const statx::distributions::evd::FitType fitting_type =
                (fitting_method == theia::MLE) ? statx::distributions::evd::MLE
                                        : statx::distributions::evd::QUANTILE_NLS;
        const bool gev_success = gevfit(negated_second_smallest_distances,
                                        &CHECK_NOTNULL(mixture_model_parameters)->mu,
                                        &mixture_model_parameters->sigma,
                                        &mixture_model_parameters->xi,
                                        fitting_type);
        std::cout << "GEV distribution: mu=" << mixture_model_parameters->mu
                << " sigma=" << mixture_model_parameters->sigma
                << " xi=" << mixture_model_parameters->xi << " flag: " << gev_success << '\n';
        return gev_success;
    }

    bool EstimateInlierRatio(
            const double inlier_ratio_upper_bound,
            const std::vector<double>& smallest_distances,
            MixtureModelParams* mixture_model_params) {
        PrimalDualQP::Params qp_params;

        // Set inequality constraints.
        SetInequalityConstraints(inlier_ratio_upper_bound, &qp_params);

        // Comppute equality constraints.
        SetEqualityConstraints(&qp_params);

        // Set the bulding QP params.
        SetQPCostFunctionParams(
                smallest_distances, *mixture_model_params, &qp_params);

        // Solve the QP.
        return SolveQPProblem(
                inlier_ratio_upper_bound, qp_params, mixture_model_params);
    }

    void SetInequalityConstraints(
            const double inlier_ratio_upper_bound, PrimalDualQP::Params* qp_params) {
        CHECK_NOTNULL(qp_params)->Ain.setConstant(0.0);
        qp_params->Ain(0, 0) = -1.0;
        qp_params->Ain(1, 1) = -1.0;
        qp_params->Ain(2, 0) = 1.0;
        qp_params->Ain(3, 1) = 1.0;

        qp_params->bin.setConstant(0.0);
        qp_params->bin(2) = inlier_ratio_upper_bound;
        qp_params->bin(3) = 1.0;
    }

    void SetEqualityConstraints(
            PrimalDualQP::Params* qp_params) {
        CHECK_NOTNULL(qp_params)->Aeq.setConstant(1.0);
        qp_params->beq(0, 0) = 1.0;
    }

    bool SolveQPProblem(
            const double inlier_ratio_upper_bound,
            const PrimalDualQP::Params& qp_params,
            MixtureModelParams* mixture_model_params) {
        Eigen::Vector2d unknowns_vector;
        // Setting initial point, i.e., guessed solution.
        unknowns_vector(0) = inlier_ratio_upper_bound / 2.0;
        unknowns_vector(1) = 1 - unknowns_vector(0);
        PrimalDualQP qp_solver;
        qp_solver.options.max_iter_ = 100;
        double min_value;
        const optimo::solvers::TERMINATION_TYPE termination_type =
                qp_solver(qp_params, &unknowns_vector, &min_value);
        std::cout << "estimated inlier ratio=" << unknowns_vector(0)
                << " termination_type: " << termination_type << '\n';
        mixture_model_params->inlier_ratio = unknowns_vector(0);
        return termination_type == optimo::solvers::SOLVED;
    }

    void SetQPCostFunctionParams(
            const std::vector<double>& smallest_distances,
            const MixtureModelParams& mixture_model_params,
            PrimalDualQP::Params* qp_params) {
        std::vector<double> empirical_cdf_vector;
        std::vector<double> empirical_cdf_support;
        statx::utils::ecdf(
                smallest_distances, &empirical_cdf_vector, &empirical_cdf_support);
        Eigen::Map<Eigen::VectorXd> empirical_cdf(&empirical_cdf_vector[0],
                                                  empirical_cdf_vector.size());

        // Calculate matrix A.
        Eigen::MatrixXd A(empirical_cdf_support.size(), 2);

        for (int i = 0; i < empirical_cdf_support.size(); i++) {
            A(i, 0) = statx::distributions::gammacdf(empirical_cdf_support[i],
                                                     mixture_model_params.k,
                                                     mixture_model_params.theta);
            A(i, 1) =
                    1.0 - statx::distributions::evd::gevcdf(-empirical_cdf_support[i],
                                                            mixture_model_params.mu,
                                                            mixture_model_params.sigma,
                                                            mixture_model_params.xi);
        }
        CHECK_NOTNULL(qp_params)->Q = A.transpose() * A;
        qp_params->d = -1.0 * A.transpose() * empirical_cdf;
    }

    void getSample (int *sample, int npoints, int total_points) {
        std::vector<int> random_numbers;
        for (int i = 0; i < npoints; i++) {
            int rand_number;
            // Generate a random number that has not already been used.
            while (std::find(random_numbers.begin(),
                             random_numbers.end(),
                             (rand_number = (*correspondence_sampler_)(rng_))) !=
                   random_numbers.end()) {
            }

            random_numbers.push_back(rand_number);
            sample[i] = rand_number;
        }
    }
};

#endif //USAC_EVSACSAMPLER_H
