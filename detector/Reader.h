#ifndef READPOINTS_READPOINTS_H
#define READPOINTS_READPOINTS_H

#include <opencv2/core/mat.hpp>

class Reader {
public:
    static void read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename);
    static void getInliers (const std::string &filename, std::vector<int> &inliers);
    static void getMatrix3x3 (const std::string &filename, cv::Mat &model);
    static bool LoadPointsFromFile(cv::Mat &points, const char* file);
    static bool SavePointsToFile(const cv::Mat &points, const char* file, std::vector<int> *inliers);
    static void getPointsNby6 (const std::string& filename, cv::Mat &points);
    static void readEVDpoints (cv::Mat &points, const std::string &filename);
};
#endif //READPOINTS_READPOINTS_H