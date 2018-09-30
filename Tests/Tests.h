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

        Ransac ransac (*model, *sampler, *termination_criteria, *quality);
        double time = 0;
        for (int i = 0; i < N; i++) {
            ransac.run(points, estimator);
            time += ransac.getQuality()->getComputationTime();
        }
        std::cout << "average time of "<< N <<" runs is " << (time/N) << "mcs using " << model->model_name
                  << " points size is " << points.size().width << "\n";
    }

    //todo add functions for test () and storeResults ()

};

#endif //TESTS_TESTS_H