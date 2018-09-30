#include <opencv2/core/mat.hpp>
#include "Tests/Tests.h"
#include "RandomGenerator/EvaluateRandomGenerators.h"

int main (int args, char ** argv) {
    Tests tests;

    // evaluating random generators
//    evaluateRandomGenerators();

    // run tests
//     tests.testLineFitting();
//     tests.testHomographyFitting();
     tests.testFundamentalFitting();
//     tests.testEssentialFitting();

	return 0;
}
