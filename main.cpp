#include <opencv2/core/mat.hpp>
#include "Tests/Tests.h"
#include "RandomGenerator/EvaluateRandomGenerators.h"
#include "Usac/Ransac/RansacOutput.h"
#include "Usac/Utils/NearestNeighbors.h"

int main (int args, char ** argv) {
    Tests tests;

    // evaluating random generators
   // evaluateRandomGenerators();
//
//    NearestNeighbors nn;
//    nn.test();
    // run tests
    // tests.testLineFitting();
    tests.testHomographyFitting();
//     tests.testFundamentalFitting();
//     tests.testEssentialFitting();

	return 0;
}
