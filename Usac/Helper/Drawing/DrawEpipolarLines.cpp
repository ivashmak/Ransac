#include "Drawing.h"

void drawEpipolarLines_ (cv::Mat &img1, cv::Mat& img2, const cv::Mat& lines, const cv::Mat& pts1, const cv::Mat& pts2) {
    int c = img1.cols, r = img1.rows;
    float x0, y0, x1, y1;
    float  r0, r1, r2;

    int lines_to_draw = std::min (50, pts1.rows);
    for (int i = 0; i < lines_to_draw; i++) {
        r0 = lines.at<float>(i, 0);
        r1 = lines.at<float>(i, 1);
        r2 = lines.at<float>(i, 2);

        x0 = 0;
        x1 = c;

        y0 = -r2/r1;
        y1 = -(r2 + r0*c)/r1;

        cv::Scalar color = cv::Scalar(random()%255, random ()%255, random ()%255);
        cv::line(img1, cv::Point_<float> (x0, y0), cv::Point_<float> (x1, y1), color);
        cv::circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
        cv::circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);
    }
}

/*
 * Draw epipolar line for Fundamental matrix
 */
void Drawing::drawEpipolarLines (const std::string& img_name, cv::InputArray points1, cv::InputArray points2, const cv::Mat& F) {
    cv::Mat pts1 = points1.getMat(), pts2 = points2.getMat();

    cv::hconcat(pts1, cv::Mat_<float>::ones (pts1.rows, 1), pts1);
    cv::hconcat(pts2, cv::Mat_<float>::ones (pts2.rows, 1), pts2);

    /*
     * For every point in one of the two images of a stereo pair,
     * the function finds the equation of the corresponding epipolar
     * line in the other image.
     */
    cv::Mat lines1, lines2;
    cv::computeCorrespondEpilines(pts2, 2, F, lines1);
    cv::computeCorrespondEpilines(pts1, 1, F, lines2);

    std::string folder = "../dataset/adelaidermf/";
//    std::string folder = "../dataset/Lebeda/kusvod2/";

    cv::Mat img1 = cv::imread(folder + img_name + "A.png"),
            img2 = cv::imread(folder + img_name + "B.png");

    Drawing::drawing_resize(img1);
    Drawing::drawing_resize(img2);

    cv::Mat img3 = img1.clone(), img4 = img2.clone();

    drawEpipolarLines_ (img1, img2, lines1, pts1, pts2);
    drawEpipolarLines_ (img4, img3, lines2, pts2, pts1);

    cv::hconcat(img1, img4, img1);

    cv::resize(img1, img1, cv::Size(0.8*img1.cols, img1.rows));

    imshow("Epipolar lines using Fundamental matrix 1", img1);

    cv::waitKey (0);
}
