#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "ReadPoints.h"

/*
 * Get correspondence points from file
 * Assume syntax as
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename) {
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

void getPointsNby6 (const std::string& filename, cv::Mat &points) {
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
void getInliers (const std::string &filename, std::vector<int> &inliers) {
    std::fstream file(filename, std::ios_base::in);

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
void getMatrix3x3 (const std::string &filename, cv::Mat &model) {
    model = cv::Mat_<float>(3,3);
    std::fstream file(filename, std::ios_base::in);

    float val;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file >> val;
            model.at<float>(i,j) = val;
        }
    }

}



bool SavePointsToFile(const cv::Mat &points, const char* file, std::vector<int> *inliers)
{
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

bool LoadPointsFromFile(cv::Mat &points, const char* file)
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
void readEVDpoints (cv::Mat &points, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, x2, y2;

    std::string tmp_str;
    cv::Mat tmp;

    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str, ',');
    std::getline(file, tmp_str);

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

        tmp = (cv::Mat_<float>(1, 4) << x1, y1, x2, y2);
        points.push_back(tmp);
    }
    file.close();
}