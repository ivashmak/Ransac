#include "tests/Tests.h"

int main (int args, char ** argv) {
    Tests tests;

    tests.testRansac();

    tests.testHomographies();
	return 0;
}
