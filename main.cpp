#include <ctime>
#include <cstdlib>
#include "Tests/Tests.h"
#include "Usac/RandomGenerator/ArrayRandomGenerator.h"


int main (int args, char ** argv) {
    Tests tests;

    ArrayRandomGenerator gr;
    gr.calculateEntropy(10000);

    // run tests
    tests.testLineFitting();
//     tests.testHomographyFitting();
//     tests.testEssentialFitting();
//     tests.testFundamentalFitting();

	return 0;
}
