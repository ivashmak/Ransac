#include "Ransac.h"
#include "../LocalOptimization/RansacLocalOptimization.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}

void Ransac::run(cv::InputArray input_points, bool LO) {
    /*
     * Check if all components are initialized and safe to run
     * todo: add more criteria
     */
    assert(!input_points.empty());
    assert(estimator != nullptr);
    assert(model != nullptr);
    assert(quality != NULL);
    assert(sampler != nullptr);
    assert(termination_criteria != nullptr);
    assert(sampler->isInit());

//    std::cout << "asserted\n";

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

//    std::cout << "Points size " << points_size << '\n';

    // initialize termination criteria
    termination_criteria->init(model);

    int iters = 0;
    int max_iters = model->max_iterations;

    Score *best_score = new Score, *current_score = new Score;
    best_score->inlier_number = 0;
    best_score->score = 0;

    int *sample = new int[estimator->SampleNumber()];

    Model **models = new Model*[1];
    Model *non_minimal_model = new Model(*model);
    Model *best_model = new Model (*model);
    models[0] = model;

//    std::cout << "begin\n";

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int *inliers = new int[points_size];
    std::vector<int> max_inliers;
    LocalOptimization * lo_ransac = new RansacLocalOptimization (*model, *sampler, *termination_criteria, *quality, *estimator);
    LO = true;
    while (iters < max_iters) {
        sampler->generateSample(sample);

        int number_of_models = estimator->EstimateModel(sample, models);

        for (int i = 0; i < number_of_models; i++) {
//            std::cout << i << "-th model\n";

            estimator->setModelParameters(models[i]);

            // we need inliers only for local optimization
            quality->GetModelScore(estimator, models[i], input_points, points_size, *current_score, inliers, LO);

            if (quality->IsBetter(current_score, best_score)) {

                if (LO) {
                    Score *lo_score = new Score;
                    Model *lo_model = new Model (*model);
                    lo_ransac->GetLOModelScore (*lo_model, *lo_score, current_score, input_points, points_size, inliers);

                    std::cout << "lo score " << lo_score->inlier_number << '\n';
                    std::cout << "curr score " << current_score->inlier_number << '\n';

                }

                // copy current score to best score
                best_score->inlier_number = current_score->inlier_number;
                best_score->score = current_score->score;

                // remember best model
                best_model->setDescriptor (models[i]->returnDescriptor());

                max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, points_size);
//            std::cout << "max iters prediction " << max_iters << '\n';
            }
        }

//        std::cout << "current iteration " << iters << '\n';

        iters++;
    }

    /*
     * if we did not use local optimization, so we did not get any inliers.
     */
    if (!LO) {
        max_inliers = std::vector<int>(best_score->inlier_number);
        quality->getInliers(estimator, points_size, best_model, max_inliers);
    }

    estimator->EstimateModelNonMinimalSample(&max_inliers[0], best_score->inlier_number, *non_minimal_model);

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;

    // store quality results
    ransac_output = new RansacOutput (best_model, non_minimal_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(), best_score->inlier_number, iters);

    delete sample, current_score, best_score; // non_minimal_model, best_model
}
