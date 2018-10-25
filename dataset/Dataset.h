#ifndef USAC_DATASET_H
#define USAC_DATASET_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>

void saveDatasetToFiles ();
std::vector<std::vector<std::string>> getEVDfilenames (bool reset_files=false);
std::vector<std::string> getHomographyDatasetPoints (bool reset_files=false);
std::vector<std::string> getFundamentalDatasetPoints (bool reset_files=false);



#endif //USAC_DATASET_H
