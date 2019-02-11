#include "Dataset.h"

void Dataset::saveDatasetToFiles () {
    FILE *fp1, *fp2;
    std::string evd_path1 = "../dataset/EVD/1/";
    std::string evd_path2 = "../dataset/EVD/2/";

    std::string command1 = "ls "+evd_path1+"*.jpg "+evd_path1+"*.png "+evd_path1+"*.JPEG 2> /dev/null";
    std::string command2 = "ls "+evd_path2+"*.jpg "+evd_path2+"*.png "+evd_path2+"*.JPEG 2> /dev/null";

    fp1 = popen((command1).c_str(), "r");
    fp2 = popen((command2).c_str(), "r");

    if (fp1 == nullptr || fp2 == nullptr) {
        printf("EVD dataset not found\n" );
        exit(1);
    }

    FILE *evd1 = fopen("../dataset/evd1.txt", "w");
    FILE *evd2 = fopen("../dataset/evd2.txt", "w");

    char filename1[500], filename2[500];

    while (fgets(filename1, sizeof(filename1)-1, fp1) != nullptr && fgets(filename2, sizeof(filename2)-1, fp1) != nullptr) {
        fprintf(evd1, "%s", filename1);
        fprintf(evd2, "%s", filename2);
    }

    fclose(evd1); fclose(evd2);
    pclose(fp1); pclose(fp2);
}

std::vector<std::string> Dataset::getEVDDataset () {
    return std::vector<std::string> {"adam", "cafe",
                                       "mag", "cat", "dum",     "face",
                                       "fox",        "girl",         "graf",
                                       "grand",  "index",
                                       "pkk", "shop", "there" , "vin"};

}


std::vector<std::string> Dataset::getHomographyDatasetPoints () {
    return std::vector<std::string> {"adam", "Brussels",
                                       "boat",
                                       "BostonLib",     "city",
                                       "Boston",        "Eiffel",         "WhiteBoard",
                                       "BruggeSquare",  "ExtremeZoom",
                                       "BruggeTower",   "graf"};

//    std::vector<std::string> problem_img = getProblemHomographyDatasetPoints();
//    for (int i = 0; i < problem_img.size(); i++) {
//        fnames.push_back(problem_img[i]);
//    }
};

std::vector<std::string> Dataset::getProblemHomographyDatasetPoints () {
    return std::vector<std::string> {"LePoint1", "LePoint2", "LePoint3", "CapitalRegion"};
};


std::vector<std::string> Dataset::getKusvod2Dataset () {
    return std::vector<std::string>  {"booksh", "box",
                                       "castle", "corr", "graff",
                                       "head", "kampa", "Kyoto",
                                       "leafs", "plant", "rotunda",
                                       "shout", "valbonne", "wall",
                                       "wash", "zoom"};
};

std::vector<std::string> Dataset::getAdelaidermfDataset () {
    return std::vector<std::string> {
//            "barrsmith",
            "bonhall",
//            "bonython",
            "elderhallb",
            "hartley",
            "johnsona",
            "johnsonb",
            "ladysymon",
            "library",
            "napiera",
            "napierb",
            "neem",
            "nese",
            "oldclassicswing",
            "physics",
            "sene",
            "unihouse",
//            "unionhouse"
    };
};

std::vector<std::string> Dataset::getStrechaDataset () {
    std::vector<std::string> fnames;
    std::fstream file ("../dataset/MVS/zdataset.txt");
    if (! file.is_open()) {
        std::cout << "can read images for Strecha dataset!\n";
        exit (0);
    }
    std::string name;
    while (file >> name) {
        fnames.push_back(name);
    }
    return fnames;
}

//printf "\"%s\",\n" $(ls *.png)
std::vector<std::string> Dataset::getStrechaCastleDenseImages (int option) {
    if (option == 0) {
        return std::vector<std::string>{
                "castle_dense_0000.png", "castle_dense_0001.png", "castle_dense_0002.png",
                "castle_dense_0003.png", "castle_dense_0004.png", "castle_dense_0005.png",
                "castle_dense_0006.png", "castle_dense_0007.png", "castle_dense_0008.png",
                "castle_dense_0009.png", "castle_dense_0010.png", "castle_dense_0011.png",
                "castle_dense_0012.png", "castle_dense_0013.png", "castle_dense_0014.png",
                "castle_dense_0015.png", "castle_dense_0016.png", "castle_dense_0017.png",
                "castle_dense_0018.png"
        };
    } else if (option == 1) {
        return std::vector<std::string>  {
                "castle_dense_large_0000.png", "castle_dense_large_0001.png", "castle_dense_large_0002.png",
                "castle_dense_large_0003.png", "castle_dense_large_0004.png", "castle_dense_large_0005.png",
                "castle_dense_large_0006.png", "castle_dense_large_0007.png", "castle_dense_large_0008.png",
                "castle_dense_large_0009.png", "castle_dense_large_0010.png", "castle_dense_large_0011.png",
                "castle_dense_large_0012.png", "castle_dense_large_0013.png", "castle_dense_large_0014.png",
                "castle_dense_large_0015.png", "castle_dense_large_0016.png", "castle_dense_large_0017.png",
                "castle_dense_large_0018.png", "castle_dense_large_0019.png", "castle_dense_large_0020.png",
                "castle_dense_large_0021.png", "castle_dense_large_0022.png", "castle_dense_large_0023.png",
                "castle_dense_large_0024.png", "castle_dense_large_0025.png", "castle_dense_large_0026.png",
                "castle_dense_large_0027.png", "castle_dense_large_0028.png", "castle_dense_large_0029.png"
        };
    } else if (option == 2) {
        return std::vector<std::string>  {
                "castle_entry_dense_0000.png",
                "castle_entry_dense_0001.png",
                "castle_entry_dense_0002.png",
                "castle_entry_dense_0003.png",
                "castle_entry_dense_0004.png",
                "castle_entry_dense_0005.png",
                "castle_entry_dense_0006.png",
                "castle_entry_dense_0007.png",
                "castle_entry_dense_0008.png",
                "castle_entry_dense_0009.png"
        };
    } else if (option == 3) {
        return std::vector<std::string> {
                "fountain_dense_0000.png", "fountain_dense_0001.png", "fountain_dense_0002.png",
                "fountain_dense_0003.png", "fountain_dense_0004.png", "fountain_dense_0005.png",
                "fountain_dense_0006.png", "fountain_dense_0007.png", "fountain_dense_0008.png",
                "fountain_dense_0009.png", "fountain_dense_0010.png"
        };
    } else if (option == 4) {
        return std::vector<std::string>{
                "herzjesu_dense_0000.png", "herzjesu_dense_0001.png", "herzjesu_dense_0002.png",
                "herzjesu_dense_0003.png", "herzjesu_dense_0004.png", "herzjesu_dense_0005.png",
                "herzjesu_dense_0006.png", "herzjesu_dense_0007.png"
        };
    } else if (option == 5) {
        return std::vector<std::string>{
                "herzjesu_dense_large_0000.png", "herzjesu_dense_large_0001.png", "herzjesu_dense_large_0002.png",
                "herzjesu_dense_large_0003.png", "herzjesu_dense_large_0004.png", "herzjesu_dense_large_0005.png",
                "herzjesu_dense_large_0006.png", "herzjesu_dense_large_0007.png", "herzjesu_dense_large_0008.png",
                "herzjesu_dense_large_0009.png", "herzjesu_dense_large_0010.png", "herzjesu_dense_large_0011.png",
                "herzjesu_dense_large_0012.png", "herzjesu_dense_large_0013.png", "herzjesu_dense_large_0014.png",
                "herzjesu_dense_large_0015.png", "herzjesu_dense_large_0016.png", "herzjesu_dense_large_0017.png",
                "herzjesu_dense_large_0018.png", "herzjesu_dense_large_0019.png", "herzjesu_dense_large_0020.png",
                "herzjesu_dense_large_0021.png", "herzjesu_dense_large_0022.png", "herzjesu_dense_large_0023.png",
                "herzjesu_dense_large_0024.png",
        };
    } else {
        std::cout <<"undefined option for strecha dataset\n";
        return std::vector<std::string>();
    }
}


std::vector<std::string> Dataset::getSyntecticLine2dDataset () {
    return std::vector<std::string> {
        "w=1000_h=1000_n=3.000000_I=200_N=10200",
        "w=1000_h=1000_n=3.000000_I=500_N=10500",
        "w=1000_h=1200_n=3.000000_I=200_N=10200",
        "w=1000_h=1200_n=3.000000_I=500_N=10500",
        "w=1200_h=1000_n=3.000000_I=200_N=10200",
        "w=1200_h=1000_n=3.000000_I=500_N=10500",
        "w=1200_h=1200_n=3.000000_I=200_N=10200",
        "w=1200_h=1200_n=3.000000_I=500_N=10500"
    };
}
