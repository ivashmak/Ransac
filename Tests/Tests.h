#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H


#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Quality/Quality.h"
#include "../Usac/Ransac/Ransac.h"

class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    /*
     * Display average results such as computational time,
     * number of inliers of N runs of Ransac.
     */
    void getAverageResults (cv::InputArray points,
                    Estimator *const estimator,
                    Model *const model,
                    Sampler *const sampler,
                    TerminationCriteria * const termination_criteria,
                    Quality *const quality,
                    int N,
                    bool LO) {

        
        int bad_models_counter = 0;
        double time = 0;
        float num_inliers = 0;
        for (int i = 0; i < N; i++) {
            Ransac ransac (*model, *sampler, *termination_criteria, *quality, *estimator);
            ransac.run(points, LO);
            RansacOutput *ransacOutput = ransac.getRansacOutput();
            time += ransacOutput->getTimeMicroSeconds();
            num_inliers += ransacOutput->getNumberOfInliers();
        }
        std::cout << N << " runs of Ransac " << " for model estimation " << model->name << " with points size " << points.size().width << '\n';
        std::cout << "\tAverage time of is " << (time/N) << "mcs.\n";
        std::cout << "\tAverage number of inliers is " << (num_inliers/N) << ".\n";

        //        std::cout << "bad models " << bad_models_counter << '\n';
    }

    //todo add functions for test (), storeResults () and showResults


    void test () {

    }

    void storeResults () {

    }

    void showResults (RansacOutput * ransacOutput) {

    }
};

#endif //TESTS_TESTS_H