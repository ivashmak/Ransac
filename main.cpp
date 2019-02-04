#include <opencv2/core/mat.hpp>
#include "Tests/Tests.h"
#include "usac/random_generator/EvaluateRandomGenerators.h"
#include "usac/ransac/ransac_output.hpp"
#include "usac/utils/nearest_neighbors.hpp"

#include "usac/sampler/prosac_sampler.hpp"
#include "usac/termination_criteria/prosac_termination_criteria.hpp"
#include "dataset/SaveGTModel.h"
#include "usac/sampler/prosac_simple_sampler.hpp"

int main (int args, char ** argv) {
    Tests tests;

//    saveGTModel(DATASET::Adelaidermf);

    // evaluating random generators
   // evaluateRandomGenerators();


    // test calculating nearest neighbors using flann and nano flann
//    NearestNeighbors nn;
//    nn.test();


//    generate_syntectic_dataset();
//    saveGTModelKusvod2();
//    tests.testNeighborsSearch();

//     run tests
//  tests.testLineFitting();
//     tests.testHomographyFitting();
     tests.testFundamentalFitting();
//    tests.testEssentialFitting();

	return 0;
}
