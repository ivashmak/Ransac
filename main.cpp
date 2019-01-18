#include <opencv2/core/mat.hpp>
#include "Tests/Tests.h"
#include "RandomGenerator/EvaluateRandomGenerators.h"
#include "Usac/Ransac/RansacOutput.h"
#include "Usac/Utils/NearestNeighbors.h"

#include "Usac/Sampler/ProsacSampler.h"
#include "Usac/TerminationCriteria/ProsacTerminationCriteria.h"
#include "dataset/SaveGTModel.h"
#include "Usac/Sampler/ProsacSimpleSampler.h"

int main (int args, char ** argv) {
    Tests tests;

//    saveGTModel(DATASET::Adelaidermf);

    // evaluating random generators
   // evaluateRandomGenerators();


    // test calculating nearest neighbors using flann and nano flann
//    NearestNeighbors nn;
//    nn.test();


//    generate_syntectic_dataset();

    tests.testNeighborsSearch();

//     run tests
//   tests.testLineFitting();
//     tests.testHomographyFitting();
//    tests.testFundamentalFitting();
//    tests.testEssentialFitting();

	return 0;
}
