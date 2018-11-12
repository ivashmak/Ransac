#include "Ransac.h"
#include "../LocalOptimization/RansacLocalOptimization.h"
#include "../Estimator/DLT/DLT.h"
#include "../LocalOptimization/GraphCut.h"
#include "../SPRT.h"

int getPointsSize (cv::InputArray points) {
//    std::cout << points.getMat(0).total() << '\n';

    if (points.isVector()) {
        return points.size().width;
    } else {
        return points.getMat().rows;
    }
}


void Ransac::run(cv::InputArray input_points) {
    // todo: initialize (= new) estimator, quality, sampler and others here...

    /*
     * Check if all components are initialized and safe to run
     * todo: add more criteria
     */
    assert(!input_points.empty());
    assert(estimator != nullptr);
    assert(model != nullptr);
    assert(quality != nullptr);
    assert(sampler != nullptr);
    assert(termination_criteria != nullptr);
    assert(sampler->isInit());

    auto begin_time = std::chrono::steady_clock::now();

    int points_size = getPointsSize(input_points);

//   std::cout << "Points size " << points_size << '\n';

    // initialize termination criteria
    termination_criteria->init(model);

    Score *best_score = new Score, *current_score = new Score;

    std::vector<Model*> models;
    models.push_back (model);

    Model *best_model = new Model;
    best_model->copyFrom (model);

    // get information from model about LO
    bool LO = model->LO;
    bool GraphCutLO = model->GraphCutLO;
    bool SprtLO = model->SprtLO;
    
    std::cout << "General ransac Local Optimization " << LO << "\n";
    std::cout << "graphCut Local Optimization " << GraphCutLO << "\n";
    std::cout << "Sprt Local Optimization " << SprtLO << "\n";    

    /*
     * Allocate inliers of points_size, to avoid reallocation in getModelScore()
     */
    int * max_inliers = new int[points_size];
    int * inliers = new int[points_size];
    int * sample = new int[estimator->SampleNumber()];

    bool best_LO_model = false;
    unsigned int lo_runs = 0;
    unsigned int lo_iterations = 0;
    unsigned int number_of_models;

    LocalOptimization * lo_ransac;
    Score *lo_score;
    Model *lo_model;   
    if (LO) {
        lo_score = new Score;
        lo_model = new Model; lo_model->copyFrom(model);
        lo_ransac = new RansacLocalOptimization (model, sampler, termination_criteria, quality, estimator);
    }

    SPRT * sprt;
    if (SprtLO) {
        sprt = new SPRT;
        sprt->initialize(model, points_size);
    }

    // Graph cut local optimization
    GraphCut * graphCut;
    if (GraphCutLO) {
        graphCut = new GraphCut;
    }

    int iters = 0;
    int max_iters = model->max_iterations;

    while (iters < max_iters) {

        sampler->generateSample(sample);
        // std::cout << "samples are generated\n";

        number_of_models = estimator->EstimateModel(sample, models);

        // std::cout << "minimal model estimated\n";

        for (int i = 0; i < number_of_models; i++) {
            // std::cout << i << "-th model\n";

                if (GraphCutLO) {
                    // if LO is true than get inliers   
                    // we need inliers only for local optimization
                    if (LO) {
                        graphCut->labeling(neighbors, estimator, models[i], inliers, *current_score,
                        points_size, true);    
                    } else {
                        graphCut->labeling(neighbors, estimator, models[i], nullptr, *current_score,
                        points_size, false);    
                    }
                    
                    if (SprtLO) {
                        // we don't no model score, no inliers
                        sprt->verifyModelAndGetModelScore(estimator, models[i], iters, best_score->inlier_number, 
                            false, nullptr, false, nullptr);        
                    }
                } else 
                if (SprtLO) {
                    if (LO) {
                        sprt->verifyModelAndGetModelScore(estimator, models[i], iters, best_score->inlier_number,
                        true, current_score, true, inliers);        
                    } else {
                        sprt->verifyModelAndGetModelScore(estimator, models[i], iters, best_score->inlier_number,
                        true, current_score, false, nullptr);      
                    }
                } else {
                    if (LO) {
                        quality->GetModelScore(estimator, models[i], input_points, points_size, *current_score, 
                        inliers, true);            
                    } else {
                        quality->GetModelScore(estimator, models[i], input_points, points_size, *current_score, 
                        nullptr, false);            
                    }
                }

                // }
            // std::cout << "general " << current_score->inlier_number << "\n";
            
            if (*current_score > best_score) {

                 // std::cout << "current score = " << current_score->score << '\n';

                if (LO) {
                    bool can_finish;
                    unsigned int lo_iters = lo_ransac->GetLOModelScore (*lo_model, *lo_score,
                            current_score, input_points, points_size, iters, inliers, &can_finish);

                     // std::cout << "lo iterations " << lo_iters << '\n';
                     // std::cout << "lo score " << lo_score->inlier_number << '\n';
                     // std::cout << "curr score " << current_score->inlier_number << '\n';
                     // std::cout << "lo model best found " << lo_model->returnDescriptor() << '\n';
                    if (*lo_score > current_score) {
                        // std::cout << "LO score is better than current score\n";
                        best_score->copyFrom(lo_score);
                        best_LO_model = true;

                        best_model->setDescriptor(lo_model->returnDescriptor());
                    } else{
                        best_score->copyFrom(current_score);
                        best_LO_model = false;

                        best_model->setDescriptor(models[i]->returnDescriptor());
                    }

                    // std::cout << "best model descriptor " << best_model->returnDescriptor() << '\n';
                    // std::cout << "best score " << best_score->inlier_number << '\n';

                    iters += lo_iters;

                    // no need, just for experiments
                    lo_iterations += lo_iters;
                    lo_runs++;
                    //

                    if (can_finish) {
                        iters++;
                        // std::cout << "CAN FINISH\n";
                        goto end;
                    }
                } else {

                    // copy current score to best score
                    best_score->inlier_number = current_score->inlier_number;
                    best_score->score = current_score->score;

                    // std::cout << "best score inlier number " << best_score->inlier_number << '\n';

                    // remember best model
                    best_model->setDescriptor (models[i]->returnDescriptor());
                }
                
                if (SprtLO) {
                    max_iters = sprt->getMaximumIterations(current_score->inlier_number);
                } else {
                    max_iters = termination_criteria->getUpBoundIterations(best_score->inlier_number, points_size);      
                }

                // std::cout << "max iters prediction = " << max_iters << '\n';
            }
            // std::cout << "current iteration = " << iters << '\n';
            iters++;
        }
    }

    end:

    /*
     * If best model is LO model, so lo used non minimal model estimation
     * so we don't need to run it again. And model will be equal to non minimal model.
     */
    if (!best_LO_model) {
        // std::cout << "Calculate Non minimal model\n";
        Model *non_minimal_model = new Model;
        non_minimal_model->copyFrom (model);
    
        // get inliers from best model
        if (GraphCutLO) {
            graphCut->labeling(neighbors, estimator, best_model, max_inliers, *current_score, points_size, true);
        } else {
            quality->getInliers(estimator, points_size, best_model, max_inliers);
        }
            
        // estimate non minimal model with max inliers
        estimator->EstimateModelNonMinimalSample(max_inliers, best_score->inlier_number, *non_minimal_model);
        quality->GetModelScore(estimator, non_minimal_model, input_points, points_size, *current_score, 
            nullptr, false);

        // Priority is for non minimal model estimation
//        if (current_score->inlier_number >= best_score->inlier_number) {
            best_score->copyFrom(current_score);
            best_model->setDescriptor(non_minimal_model->returnDescriptor());
//        } else {
                   if (current_score->inlier_number < best_score->inlier_number)
           std::cout
                   << "\033[1;31mNon minimal model worse than best ransac model. May be something wrong. Check it!\033[0m \n";
           std::cout << "end non minimal score " << current_score->inlier_number << '\n';
           std::cout << "end best score " << best_score->inlier_number << '\n';
//        }
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> fs = end_time - begin_time;
    // here is ending ransac main implementation

    // get final inliers of the best model
    quality->getInliers(estimator, points_size, best_model, max_inliers);
    float average_error = quality->getAverageError(estimator, best_model, max_inliers, best_score->inlier_number);

    // Store results
    ransac_output = new RansacOutput (best_model, max_inliers,
            std::chrono::duration_cast<std::chrono::microseconds>(fs).count(), average_error, best_score->inlier_number, iters, lo_iterations, lo_runs);

    if (LO) {
        delete lo_ransac, lo_model, lo_score;
    }
    if (GraphCutLO) {
        delete graphCut;
    }
    if (SprtLO) {
        delete sprt;
    }
    delete sample, current_score, best_score, inliers, max_inliers, best_model;
}


// ---------- for debug ----------------------
//            Drawing drawing;
//            cv::Mat img = cv::imread ("../dataset/image1.jpg");
//            // std::vector<int> inl;
//            // for (int i = 0; i < lo_score->inlier_number; i++) {
//            //     inl.push_back(lo_inliers[i]);
//            // }
//            // drawing.showInliers(input_points, inl, img);
//            cv::Point_<float> * pts = (cv::Point_<float> *) input_points.getMat().data;
//            drawing.draw_model(models[0], cv::Scalar(255, 0, 0), img, false);
//            cv::circle (img, pts[sample[0]], 3, cv::Scalar(255, 255, 0), -1);
//            cv::circle (img, pts[sample[1]], 3, cv::Scalar(255, 255, 0), -1);
//            cv::imshow("samples img", img); cv::waitKey(0);
//            cv::imwrite( "../results/"+model->model_name+"_"+std::to_string(iters)+".jpg", img);
// -------------------------------------------
//            Drawing drawing;
//            cv::Mat img = cv::imread ("../dataset/homography/boatA.png");
//            // std::vector<int> inl;
//            // for (int i = 0; i < lo_score->inlier_number; i++) {
//            //     inl.push_back(lo_inliers[i]);
//            // }
//            // drawing.showInliers(input_points, inl, img);
//            cv::imshow("H", img); cv::waitKey(0);
// -------------------------------------------