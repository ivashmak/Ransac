#include <ctime>
#include <cstdlib>
#include "Tests/Tests.h"
#include "Usac/RandomGenerator/ArrayRandomGenerator.h"
#include "Usac/RandomGenerator/SimpleRandomGenerator.h"
#include "Usac/RandomGenerator/UniformRandomGenerator.h"
#include "Usac/RandomGenerator/EvaluateRandomGenerators.h"


int main (int args, char ** argv) {
    Tests tests;

    // evaluating random generators
//    evaluateRandomGenerators();

    // run tests
    tests.testLineFitting();
//     tests.testHomographyFitting();
//     tests.testEssentialFitting();
//     tests.testFundamentalFitting();

	return 0;
}
