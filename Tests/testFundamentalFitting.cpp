#include "Tests.h"

#include "../Usac/FundamentalMatrixEstimation.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing.h"

void testFundamentalMatrixEstimation (cv::InputArray points1, cv::InputArray points2);
static Drawing drawing;

void Tests::testFundamentalFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2);

    testFundamentalMatrixEstimation(points1, points2);
}

void testFundamentalMatrixEstimation (cv::InputArray points1, cv::InputArray points2) {
    FundamentalMatrixEstimation * fun_mat_est = new FundamentalMatrixEstimation;
    cv::Mat F, F_opencv;

    fun_mat_est->sevenPointsAlg(points1, points2, F);
    fun_mat_est->sevenPointsOpencv(points1, points2, F_opencv);

    std::cout << "Fundamental Matrix =\n " << F << "\n\n";
    std::cout << "Fundamental Matrix OpenCV =\n " << F_opencv << "\n\n";

    drawing.drawEpipolarLines(points1, points2, F);
}