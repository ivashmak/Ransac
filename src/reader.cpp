// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/reader.hpp"

/*
 * Get correspondence points from file
 * Assume syntax as
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void cv::usac::Reader::read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2, inl;
    cv::Mat tmp = cv::Mat_<float>(1, 2);
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);

    }
}

/*
 * Get inliers (isinlier1, ..., isinlierN) from file with syntax
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void cv::usac::Reader::getInliers (const std::string &filename, std::vector<int> &inliers) {
    std::fstream file(filename, std::ios_base::in);
    inliers.clear();

    cv::Mat tmp = cv::Mat_<float>(1, 2), pts1, pts2;
    float x1, y1, z1, x2, y2, z2;
    int inl;
    int p = 0;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {

        if (inl > 0) {
            inliers.push_back(p);
        }
        p++;

        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);
    }
}

/*
 * Get matrix from file.
 * Assume file syntax as
 * a11 a12 a13
 * a21 a22 a23
 * a31 a32 a33
 */
void cv::usac::Reader::getMatrix3x3 (const std::string &filename, cv::Mat &model) {
    model = cv::Mat_<float>(3,3);
    std::fstream file(filename, std::ios_base::in);
    if (! file.is_open()) {
        std::cout << "Wrong direction to matrix file!\n";
        exit (0);
    }

    float val;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file >> val;
            model.at<float>(i,j) = val;
        }
    }

}

bool cv::usac::Reader::LoadPointsFromFile(cv::Mat &points, const char* file)
{
    std::ifstream infile(file);

    if (!infile.is_open())
        return false;

    int N;
    std::string line;
    int line_idx = 0;
    float *points_ptr = NULL;

    while (getline(infile, line))
    {
        if (line_idx++ == 0)
        {
            N = atoi(line.c_str());
            points = cv::Mat(N, 4, CV_32F);
            points_ptr = reinterpret_cast<float*>(points.data);
            continue;
        }

        std::istringstream split(line);
        split >> *(points_ptr++);
        split >> *(points_ptr++);
        split >> *(points_ptr++);
        split >> *(points_ptr++);
    }

    infile.close();
    return true;
}

void cv::usac::Reader::readInliers (std::vector<int>&inliers, const std::string &filename) {
    inliers.clear();
    std::fstream file(filename, std::ios_base::in);

    if (! file.is_open()) {
        std::cout << "Wrong direction to inliers file, probably for Strecha dataset. Reader::readInliers!\n";
        exit (0);
    }
    int num_inliers;
    file >> num_inliers;
    inliers = std::vector<int>(num_inliers);

    int inl_idx;
    for (int i = 0; i < num_inliers; i++) {
        file >> inl_idx;
        inliers[i] = inl_idx;
    }
}