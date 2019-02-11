#include "Drawing.h"
#include "../../usac/estimator/fundamental_estimator.hpp"
#include "../../dataset/Dataset.h"
#include "../../dataset/GetImage.h"

void drawEpipolarLines_ (cv::Mat &img1, cv::Mat &img2, const std::vector<int> &inliers, const cv::Mat& lines1, const cv::Mat& lines2, const cv::Mat& pts1, const cv::Mat& pts2) {
    int c = img1.cols, r = img1.rows;

//    int inliers_to_draw = std::min (50, (int)inliers.size());
    int inliers_to_draw = inliers.size();
    for (int i = 0; i < inliers_to_draw; i++) {
        float r1_0 = lines1.at<float>(i, 0);
        float r1_1 = lines1.at<float>(i, 1);
        float r1_2 = lines1.at<float>(i, 2);

        float r2_0 = lines2.at<float>(i, 0);
        float r2_1 = lines2.at<float>(i, 1);
        float r2_2 = lines2.at<float>(i, 2);

        float x0 = 0;
        float x1 = c;

        float y1_0 = -r1_2/r1_1;
        float y1_1 = -(r1_2 + r1_0*c)/r1_1;

        float y2_0 = -r2_2/r2_1;
        float y2_1 = -(r2_2 + r2_0*c)/r2_1;

        cv::Scalar color = cv::Scalar(random()%255, random ()%255, random ()%255);
        cv::line(img1, cv::Point_<float> (x0, y1_0), cv::Point_<float> (x1, y1_1), color);
        cv::line(img2, cv::Point_<float> (x0, y2_0), cv::Point_<float> (x1, y2_1), color);
        cv::circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
        cv::circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);

    }
}

void DrawMatches_(cv::Mat points, std::vector<int> inliers, cv::Mat image1, cv::Mat image2, cv::Mat &out_image)
{
    float rotation_angle = 0;
    bool horizontal = true;

    if (image1.cols < image1.rows)
    {
        rotation_angle = 90;
    }

    int counter = 0;
    int size = 10;

    if (horizontal)
    {
        out_image = cv::Mat(image1.rows, 2 * image1.cols, image1.type()); // Your final image

        cv::Mat roiImgResult_Left = out_image(cv::Rect(0, 0, image1.cols, image1.rows)); //Img1 will be on the left part
        cv::Mat roiImgResult_Right = out_image(cv::Rect(image1.cols, 0, image2.cols, image2.rows)); //Img2 will be on the right part, we shift the roi of img1.cols on the right

        cv::Mat roiImg1 = image1(cv::Rect(0, 0, image1.cols, image1.rows));
        cv::Mat roiImg2 = image2(cv::Rect(0, 0, image2.cols, image2.rows));

        roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
        roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult

        for (int i = 0; i < inliers.size(); ++i)
        {
            int idx = inliers[i];
            cv::Point2d pt1((double)points.at<float>(idx, 0), (double)points.at<float>(idx, 1));
            cv::Point2d pt2(image2.cols + (double)points.at<float>(idx, 2), (double)points.at<float>(idx, 3));

            cv::Scalar color(255 * (double)rand() / RAND_MAX, 255 * (double)rand() / RAND_MAX, 255 * (double)rand() / RAND_MAX);

            cv::circle(out_image, pt1, size, color, static_cast<int>(size * 0.4f));
            cv::circle(out_image, pt2, size, color, static_cast<int>(size * 0.4f));
            cv::line(out_image, pt1, pt2, color, 2);
        }
    }
    else
    {
        out_image = cv::Mat(2 * image1.rows, image1.cols, image1.type()); // Your final image

        cv::Mat roiImgResult_Left = out_image(cv::Rect(0, 0, image1.cols, image1.rows)); //Img1 will be on the left part
        cv::Mat roiImgResult_Right = out_image(cv::Rect(0, image1.rows, image2.cols, image2.rows)); //Img2 will be on the right part, we shift the roi of img1.cols on the right

        cv::Mat roiImg1 = image1(cv::Rect(0, 0, image1.cols, image1.rows));
        cv::Mat roiImg2 = image2(cv::Rect(0, 0, image2.cols, image2.rows));

        roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
        roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult

        for (int i = 0; i < inliers.size(); ++i)
        {
            int idx = inliers[i];
            cv::Point2d pt1((double)points.at<float>(idx, 0), (double)points.at<float>(idx, 1));
            cv::Point2d pt2(image2.cols + (double)points.at<float>(idx, 2), (double)points.at<float>(idx, 3));

            cv::Scalar color(255 * (double)rand() / RAND_MAX, 255 * (double)rand() / RAND_MAX, 255 * (double)rand() / RAND_MAX);
            cv::circle(out_image, pt1, size, color, static_cast<int>(size * 0.4));
            cv::circle(out_image, pt2, size, color, static_cast<int>(size * 0.4));
            cv::line(out_image, pt1, pt2, color, 2);
        }
    }

}

/*
 * Draw epipolar line for Fundamental matrix
 */
void Drawing::drawEpipolarLines (Model * model, DATASET dataset, const std::string& img_name) {
    ImageData gt_data (dataset, img_name);
    cv::Mat points = gt_data.getPoints();
    cv::Mat pts1 = points.colRange(0,2);
    cv::Mat pts2 = points.colRange(2,4);

    /*
     * For every point in one of the two images of a stereo pair,
     * the function finds the equation of the corresponding epipolar
     * line in the other image.
     */
    cv::Mat lines1, lines2;
    cv::computeCorrespondEpilines(pts2, 2, model->returnDescriptor(), lines1);
    cv::computeCorrespondEpilines(pts1, 1, model->returnDescriptor(), lines2);

    cv::Mat img1 = gt_data.getImage1();
    cv::Mat img2 = gt_data.getImage2();

    Estimator * est;
    if (model->estimator == ESTIMATOR::Essential) {
        est = new EssentialEstimator(points);
    } else { // fundamental
        est = new FundamentalEstimator(points);
    }

    std::vector<int> inliers;
    Quality::getInliers(est, model->returnDescriptor(), model->threshold, points.rows, inliers);
    cv::Mat out_image;
    DrawMatches_(points, inliers, img1, img2, out_image);

    for (int i = 0; i < points.rows; i++) {
        cv::circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 4, cv::Scalar(0,0,0), -1);
        cv::circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 4, cv::Scalar(0,0,0), -1);
    }

    drawEpipolarLines_ (img1, img2, inliers, lines1, lines2, pts1, pts2);

    cv::hconcat(img1, img2, img1);

    drawing_resize(img1);

//    imshow("Matches", out_image);
    imshow("Epipolar lines", img1);
    cv::waitKey (0);
}
