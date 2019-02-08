#include "generator.h"

void Generate2DLinePoints(float noise, int inlier_number, int outlier_number,
                             int border_x, int border_y, std::vector<cv::Point2f> &points, cv::Mat &gt_model);

void generate_syntectic_dataset () {
    std::vector<int> widths;
    widths.push_back(1000);
    widths.push_back(1200);

    std::vector<int> heights;
    heights.push_back(1000);
    heights.push_back(1200);

    std::vector<float> percents_of_inliers;
    percents_of_inliers.push_back(0.02);
    percents_of_inliers.push_back(0.05);
//    percents_of_inliers.push_back(0.1);

    std::vector<float> outliers_v;
//    outliers_v.push_back(5000);
//    outliers_v.push_back(7000);
    outliers_v.push_back(10000);

    std::vector<float> noises;
    noises.push_back(3);
//    noises.push_back(6);

    std::ofstream dataset_file;
    dataset_file.open ("../dataset/line2d/dataset.txt");

    for (auto width : widths) {
        for (auto height : heights) {
            for (auto outliers : outliers_v) {
                for (auto percent_of_inliers : percents_of_inliers) {
                    for (auto noise : noises) {
                        int inliers = outliers * percent_of_inliers;
                        int N = outliers + inliers;
                        std::vector<cv::Point2f> points(N);
                        cv::Mat_<float> gt_model;
                        Generate2DLinePoints(noise, inliers, outliers, width, height, points, gt_model);

                        std::string folder = "../dataset/line2d/";
                        std::string filename = "w="+std::to_string(width)+"_h="+std::to_string(height)+"_n="+
                                    std::to_string(noise)+"_I="+std::to_string(inliers)+"_N="+std::to_string(N);
                        std::ofstream file;
                        cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

                        file.open (folder+filename+".txt");
                        std::cout << folder+filename << "\n";
                        file << width << "\n";
                        file << height << "\n";
                        file << noise << "\n";
                        file << gt_model.at<float>(0) << " " << gt_model.at<float>(1) << " " << gt_model.at<float>(2) << "\n";
                        file << N << "\n";
                        for (auto &pt : points) {
                            file << pt.x << " " << pt.y << "\n";
                            cv::circle(image, pt, 3, cv::Scalar(0, 0, 0), -1);
                        }
                        cv::imwrite ("../dataset/line2d/"+filename+".png", image);
                        dataset_file << filename << "\n";
                    }
                }
            }
        }
    }
}

void generate (std::vector<cv::Point2f> &points_out, bool reset_time, bool getGT, cv::Mat &gt_model) {
    if (reset_time) srand (time(NULL));

    int width = 600; // Width of the image
    int height = 600; // Height of the image
    float noise = 3.0f; // Noise sigma
    int outlier_number = 3000; // 1500; // Number of points not lying on the line
    int inlier_number = 300; //1500; // Number of points from the line

    int N = outlier_number + inlier_number;
    std::vector<cv::Point2f> points(N);
    Generate2DLinePoints(noise, inlier_number, outlier_number, width, height, points, gt_model);

    // Do some clever things, e.g. draw the points
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); // Create a 600*600 image

    for (int i = 0; i < N; ++i) {
        cv::circle(image, points[i], 3, cv::Scalar(0, 0, 0), -1);
    }
    cv::imwrite ("../dataset/image1.png", image);
    
    // cv::imshow("Image", image);
    // cv::waitKey(0);
    
    points_out.swap(points);
    // std::cout << "ready\n";
    // exit (0);
}

void Generate2DLinePoints(float noise, int inlier_number, int outlier_number,
                             int border_x, int border_y, std::vector<cv::Point2f> &points, cv::Mat& gt_model) {
    // Generate random line
    float alpha = M_PI * static_cast<float>(rand()) / RAND_MAX; // A random orientation
    float normal_x = sin(alpha);
    float normal_y = cos(alpha);
    float tangent_x = -normal_y; // The tangent direction of the line (x coordinate)
    float tangent_y = normal_x; // (y coordinate)

    float center_x = border_x / 2; // A point from the line, it's now just the center of the image
    float center_y = border_y / 2;
    float c = -(normal_x * center_x + normal_y * center_y); // From implicit line equation, a * x + b * y + c = 0

//    std::cout << "a = " << normal_x << " b = " << normal_y << " " << "c = " << c << "\n";
    gt_model = (cv::Mat_<float>(1,3) << normal_x, normal_y, c);

    // Add outliers
    for (int i = 0; i < outlier_number; ++i)
    {
        // Random point in the image
        points[i].x = border_x * static_cast<float>(rand()) / RAND_MAX;
        points[i].y = border_y * static_cast<float>(rand()) / RAND_MAX;
    }

    // Add inliers
    float image_diagonal = sqrt(border_x * border_x + border_y * border_y);
    for (int i = outlier_number; i < inlier_number + outlier_number; ++i)
    {
        // We need the points to be in the images.
        // It could be implemented elegantly by checking the intersections of the line and the borders, but it is more understandable like this.
        do
        {
            float param_t = static_cast<float>(rand()) / RAND_MAX - 0.5; // From parametric line equation, point = center + t * tangent_direction
            float x = center_x + param_t * tangent_x * image_diagonal; // The x coordinate of the new point from the line
            if (x < 0 || x > border_x) // It must be in the given bounds
                continue;
            float y = center_y + param_t * tangent_y * image_diagonal; // The y coordinate of the new point from the line
            if (y < 0 || y > border_y) // It must be in the given bounds
                continue;

            // Add noise in the normal direction
            x = x + normal_x * noise * static_cast<float>(rand()) / RAND_MAX - noise / 2;
            y = y + normal_y * noise * static_cast<float>(rand()) / RAND_MAX - noise / 2;

            // Save the coordinates
            points[i].x = x;
            points[i].y = y;
            break;
        } while (1);
    }
}
