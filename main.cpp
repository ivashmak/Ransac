#include "tests/Tests.h"

int main (int args, char ** argv) {
    Tests tests;

    tests.testLineFitting();
    tests.testHomographyFitting();

	return 0;
}
