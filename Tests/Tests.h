#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H


#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Quality.h"
#include "../Usac/Ransac/Ransac.h"

class Tests {
public:
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    void runNTimes (cv::InputArray points,
                    Estimator *const estimator,
                    Model *const model,
                    Sampler *const sampler,
                    TerminationCriteria * const termination_criteria,
                    Quality *const quality,
                    int N) {

        
        int bad_models_counter = 0;
        double time = 0;
        for (int i = 0; i < N; i++) {
            Ransac ransac (*model, *sampler, *termination_criteria, *quality, *estimator);
            ransac.run(points);
            RansacOutput *ransacOutput = ransac.getRansacOutput();
            time += ransacOutput->getTimeMicroSeconds();
            if (ransacOutput->getNumberOfInliers() < 1300)
                bad_models_counter++;

        }
        std::cout << "average time of "<< N <<" runs is " << (time/N) << "mcs using " << model->model_name
                  << ". Points size is " << points.size().width << "\n";
        std::cout << "bad models " << bad_models_counter << '\n';
    }

    //todo add functions for test () and storeResults ()

};

#endif //TESTS_TESTS_H