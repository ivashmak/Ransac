#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "Reader.h"

/*
 * Get correspondence points from file
 * Assume syntax as
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void Reader::read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2, inl;
    cv::Mat tmp = cv::Mat_<float>(1, 2);
    float eps = 3;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        float * pts1_ptr = (float *) pts1.data;
        float * pts2_ptr = (float *) pts2.data;
        bool cont = false;

        /*
         * Skip repeated points
         */
//        for (int i = 0; i < pts1.rows; i++) {
//            if ((fabsf(pts1_ptr[2*i] - x1) <= eps && fabsf(pts1_ptr[2*i+1] - y1) <= eps) &&
//                (fabsf(pts2_ptr[2*i] - x2) <= eps && fabsf(pts2_ptr[2*i+1] - y2) <= eps)) {
//                cont = true;
//                break;
//            }
//        }
//
//        if (cont) continue;

        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);

    }
}

/*
 * Format of file
 * x1i y1i 1 x2i y2i 1
 *
 * Return points:
 * x1i y1i x2i y2i
 */

void Reader::getPointsNby6 (const std::string& filename, cv::Mat &points) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2;
    cv::Mat tmp;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2) {
        tmp = (cv::Mat_<float>(1, 4) << x1, y1, x2, y2);
        points.push_back(tmp);
    }
}

/*
 * Get inliers (isinlier1, ..., isinlierN) from file with syntax
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void Reader::getInliers (const std::string &filename, std::vector<int> &inliers) {
    std::fstream file(filename, std::ios_base::in);
    inliers.clear();

    cv::Mat tmp = cv::Mat_<float>(1, 2), pts1, pts2;
    float x1, y1, z1, x2, y2, z2;
    int inl;
    int p = 0;
    float eps = 3;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        float * pts1_ptr = (float *) pts1.data;
        float * pts2_ptr = (float *) pts2.data;
        bool cont = false;

        /*
         * Skip repeated points
         */
//        for (int i = 0; i < pts1.rows; i++) {
//            if ((fabsf(pts1_ptr[2*i] - x1) <= eps && fabsf(pts1_ptr[2*i+1] - y1) <= eps) &&
//                    (fabsf(pts2_ptr[2*i] - x2) <= eps && fabsf(pts2_ptr[2*i+1] - y2) <= eps)) {
//                cont = true;
//                break;
//            }
//        }
//        if (cont) continue;

//        std::cout <<  x1 << " " << y1 << " " << " " << z1 << " " << x2 << " " << y2 << " " << z2 << " " << inl << "\n";

        if (inl > 0) {
//            std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << "\n";
//            std::cout << p << "\n";
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
void Reader::getMatrix3x3 (const std::string &filename, cv::Mat &model) {
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



bool Reader::SavePointsToFile(const cv::Mat &points, const char* file, std::vector<int> *inliers) {
    std::ofstream outfile(file, std::ios::out);

    float *points_ptr = reinterpret_cast<float*>(points.data);
    const int M = points.cols;

    if (inliers == NULL)
    {
        outfile << points.rows << std::endl;
        for (auto i = 0; i < points.rows; ++i)
        {
            for (auto j = 0; j < M; ++j)
                outfile << *(points_ptr++) << " ";
            outfile << std::endl;
        }
    }
    else
    {
        outfile << inliers->size() << std::endl;
        for (auto i = 0; i < inliers->size(); ++i)
        {
            const int offset = inliers->at(i) * M;
            for (auto j = 0; j < M; ++j)
                outfile << *(points_ptr + offset + j) << " ";
            outfile << std::endl;
        }
    }

    outfile.close();

    return true;
}

bool Reader::LoadPointsFromFile(cv::Mat &points, const char* file)
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

/*
 * Format
 * x1,y1,x2,y2,FGINN_ratio,SNN_ratio,detector,descriptor,is_correct
 */
void Reader::readEVDPointsInliers (cv::Mat &points, std::vector<int>&inliers, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);
    inliers.clear();

    float x1, y1, x2, y2;
    bool is_inlier;

    std::string tmp_str;
    cv::Mat tmp;

    // skip format
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str);

    int i = 0;
    while (std::getline(file, tmp_str, ',')) {
        x1 = std::stof (tmp_str);
        std::getline(file, tmp_str, ',');
        y1 = std::stof (tmp_str);
        std::getline(file, tmp_str, ',');
        x2 = std::stof (tmp_str);
        std::getline(file, tmp_str, ',');
        y2 = std::stof (tmp_str);

        // fginn
        std::getline(file, tmp_str, ',');
        // snn
        std::getline(file, tmp_str, ',');
        // detector
        std::getline(file, tmp_str, ',');
        // descr
        std::getline(file, tmp_str, ',');
        // is correct
        std::getline(file, tmp_str);
        is_inlier = static_cast<bool>(std::stof(tmp_str));

        tmp = (cv::Mat_<float>(1, 4) << x1, y1, x2, y2);
        points.push_back(tmp);

        if (is_inlier) inliers.push_back(i);
        i++;
    }
    file.close();
}


void Reader::readProjectionMatrix (cv::Mat &P, const std::string &filename) {
    P = cv::Mat_<float>(3,4);
    std::fstream file(filename, std::ios_base::in);

    if (! file.is_open()) {
        std::cout << "Wrong direction to Projection matrix file!\n";
        exit (0);
    }

    float val;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            file >> val;
            P.at<float>(i,j) = val;
        }
    }
}

void Reader::readInliers (std::vector<int>&inliers, const std::string &filename) {
    inliers.clear();
    std::fstream file(filename, std::ios_base::in);

    if (! file.is_open()) {
        std::cout << "Wrong direction to inliers file!\n";
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