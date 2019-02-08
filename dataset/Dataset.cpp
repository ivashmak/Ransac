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
    std::vector<std::string> fnames = {"adam", "cafe",
                                       "mag", "cat", "dum",     "face",
                                       "fox",        "girl",         "graf",
                                       "grand",  "index",
                                       "pkk", "shop", "there" , "vin"};

    return fnames;
}


std::vector<std::string> Dataset::getHomographyDatasetPoints () {
    std::vector<std::string> fnames = {"adam", "Brussels",
                                       "boat",
                                       "BostonLib",     "city",
                                       "Boston",        "Eiffel",         "WhiteBoard",
                                       "BruggeSquare",  "ExtremeZoom",
                                       "BruggeTower",   "graf"};

//    std::vector<std::string> problem_img = getProblemHomographyDatasetPoints();
//    for (int i = 0; i < problem_img.size(); i++) {
//        fnames.push_back(problem_img[i]);
//    }
    return fnames;
};

std::vector<std::string> Dataset::getProblemHomographyDatasetPoints () {
    std::vector<std::string> fnames = {"LePoint1", "LePoint2", "LePoint3", "CapitalRegion"};
    return fnames;
};


std::vector<std::string> Dataset::getFundamentalDatasetPoints () {
    std::vector<std::string> fnames = {"barrsmith_pts.txt", "bonhall_pts.txt",
                                       "bonython_pts.txt", "elderhalla_pts.txt", "elderhallb_pts.txt",
                                       "hartley_pts.txt", "johnssona_pts.txt", "johnssonb_pts.txt",
                                       "ladysymon_pts.txt", "library_pts.txt", "napiera_pts.txt",
                                       "napierb_pts.txt", "neem_pts.txt", "unihouse_pts.txt",
                                       "oldclassicswing_pts.txt", "physics_pts.txt", "sene_pts.txt",
                                       "unionhouse_pts.txt"};
    return fnames;
};

std::vector<std::string> Dataset::getKusvod2Dataset () {
    std::vector<std::string> fnames = {"booksh", "box",
                                       "castle", "corr", "graff",
                                       "head", "kampa", "Kyoto",
                                       "leafs", "plant", "rotunda",
                                       "shout", "valbonne", "wall",
                                       "wash", "zoom"};
    return fnames;
};

std::vector<std::string> Dataset::getAdelaidermfDataset () {
    std::vector<std::string> fnames = {
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
    return fnames;
};

std::vector<std::string> Dataset::getStrechaDataset () {
    std::vector<std::string> fnames = {
        "castle_dense_large_0025", "castle_entry_dense_0002", "herzjesu_dense_large_0013",
        "castle_dense_large_0028", "castle_dense_large_0007", "castle_dense_large_0008",
        "castle_dense_0008", "fountain_dense_0002", "castle_dense_large_0021",
        "castle_dense_0006", "castle_dense_large_0019", "fountain_dense_0009",
        "herzjesu_dense_large_0022", "herzjesu_dense_large_0024", "castle_dense_0012",
        "herzjesu_dense_large_0007", "castle_dense_0001", "herzjesu_dense_0001",
        "castle_dense_0000", "herzjesu_dense_large_0021", "herzjesu_dense_0006",
        "herzjesu_dense_large_0012", "herzjesu_dense_0000", "herzjesu_dense_large_0016",
        "herzjesu_dense_large_0020", "castle_entry_dense_0008", "castle_dense_large_0026",
        "castle_dense_0013", "castle_dense_large_0011", "herzjesu_dense_large_0005",
        "castle_dense_large_0018", "herzjesu_dense_large_0008", "castle_dense_large_0005",
        "castle_dense_large_0006", "herzjesu_dense_large_0017", "castle_dense_large_0023",
        "castle_dense_large_0010", "castle_dense_large_0024", "herzjesu_dense_0004",
        "herzjesu_dense_large_0003", "castle_entry_dense_0005", "herzjesu_dense_large_0004",
        "herzjesu_dense_large_0019", "herzjesu_dense_large_0002", "castle_dense_0015",
        "castle_dense_0002", "herzjesu_dense_large_0000", "fountain_dense_0010",
        "castle_dense_large_0013", "fountain_dense_0004", "herzjesu_dense_0003",
        "castle_dense_0017", "herzjesu_dense_large_0010", "castle_entry_dense_0001",
        "castle_dense_0005", "castle_dense_large_0016", "herzjesu_dense_large_0023",
        "fountain_dense_0008", "castle_entry_dense_0009", "herzjesu_dense_large_0011",
        "fountain_dense_0006", "herzjesu_dense_large_0015", "castle_dense_large_0002",
        "fountain_dense_0001", "castle_dense_large_0027", "castle_entry_dense_0006",
        "castle_dense_large_0009", "herzjesu_dense_large_0006", "castle_dense_large_0029",
        "herzjesu_dense_large_0001", "castle_entry_dense_0007", "castle_dense_0009",
        "castle_dense_0007", "castle_entry_dense_0004", "castle_entry_dense_0003",
        "herzjesu_dense_0002", "castle_dense_large_0004", "castle_dense_0004",
        "castle_dense_large_0014", "castle_dense_large_0015", "herzjesu_dense_0005",
        "castle_dense_large_0022", "castle_entry_dense_0000", "castle_dense_large_0000",
        "castle_dense_large_0012", "castle_dense_large_0001", "fountain_dense_0007",
        "herzjesu_dense_0007", "castle_dense_large_0017", "castle_dense_0003",
        "herzjesu_dense_large_0014", "castle_dense_large_0003", "castle_dense_0011",
        "herzjesu_dense_large_0009", "castle_dense_0014", "castle_dense_0018",
        "castle_dense_0010", "fountain_dense_0003", "fountain_dense_0000",
        "castle_dense_0016", "fountain_dense_0005", "castle_dense_large_0020",
        "herzjesu_dense_large_0018"
    };
    return fnames;
}
