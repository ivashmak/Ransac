// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/reader.hpp"

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