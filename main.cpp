#include <ctime>
#include <cstdlib>
#include "tests/Tests.h"

int main (int args, char ** argv) {
    srand (time(NULL));

    Tests tests;

    tests.testLineFitting();
    tests.testHomographyFitting();

	return 0;
}
