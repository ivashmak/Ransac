#include <ctime>
#include <cstdlib>
#include "Tests/Tests.h"

int main (int args, char ** argv) {
    srand (time(NULL));

    Tests tests;

    tests.testLineFitting();
//    Tests.testHomographyFitting();

	return 0;
}
