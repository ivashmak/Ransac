#ifndef USAC_DATASET_H
#define USAC_DATASET_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>

enum DATASET {Homogr, Adelaidermf, Kusvod2, Syntectic, Strecha, EVD};
class Dataset {
public:
    static void saveDatasetToFiles ();
    static std::vector<std::string> getEVDDataset ();
    static std::vector<std::string> getHomographyDatasetPoints ();
    static std::vector<std::string> getProblemHomographyDatasetPoints ();
    static std::vector<std::string> getFundamentalDatasetPoints ();
    static std::vector<std::string> getKusvod2Dataset ();
    static std::vector<std::string> getAdelaidermfDataset ();
};

#endif //USAC_DATASET_H
