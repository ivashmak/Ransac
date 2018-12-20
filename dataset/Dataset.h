#ifndef USAC_DATASET_H
#define USAC_DATASET_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>

enum DATASET {Homogr, Adelaidermf, Kusvod2, Syntectic, Strecha, EVD};

std::vector<std::string> getEVDDataset ();
std::vector<std::string> getHomographyDatasetPoints ();
std::vector<std::string> getProblemHomographyDatasetPoints ();
std::vector<std::string> getFundamentalDatasetPoints ();
std::vector<std::string> getKusvod2Dataset ();
std::vector<std::string> getAdelaidermfDataset ();

#endif //USAC_DATASET_H
