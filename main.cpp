#include <opencv2/core/mat.hpp>
#include "test/tests.h"
#include "test/test_random_generator.h"
#include "usac/ransac/ransac_output.hpp"
#include "usac/utils/nearest_neighbors.hpp"

#include "usac/sampler/prosac_sampler.hpp"
#include "usac/termination_criteria/prosac_termination_criteria.hpp"
#include "dataset/SaveGTModel.h"
#include "usac/sampler/prosac_simple_sampler.hpp"
#include "usac/utils/utils.hpp"

int main (int args, char ** argv) {
//    saveGTModel(DATASET::Adelaidermf);

    // evaluating random generators
   // evaluateRandomGenerators();

    // test calculating nearest neighbors using flann and nano flann
//    NearestNeighbors::test();

//    generate_syntectic_dataset();
//    saveGTModelKusvod2();
//    Tests::testNeighborsSearch();

//     run tests
  Tests::testLineFitting();
//    Tests::testHomographyFitting();
//     Tests::testFundamentalFitting();
//     Tests::testEssentialFitting();

	return 0;
}
