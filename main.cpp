#include <ctime>
#include <cstdlib>
#include "Tests/Tests.h"

int main (int args, char ** argv) {
    Tests tests;

    tests.testLineFitting();
//     tests.testHomographyFitting();
//     tests.testEssentialFitting();
//     tests.testFundamentalFitting();

	return 0;
}
