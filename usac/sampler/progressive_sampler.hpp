// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_GRADUALNAPSAC_H
#define USAC_GRADUALNAPSAC_H

#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>
#include "sampler.hpp"
#include "../random_generator/array_random_generator.hpp"
#include "../utils/math.hpp"

/*
 * https://pdfs.semanticscholar.org/cec1/2adbb307124e0c62efbaaa870836c3846b5f.pdf
 * http://www.bmva.org/bmvc/2002/papers/164/full_164.pdf
 *
 */
class ProgressiveNapsac : public Sampler {
private:
    RandomGenerator * random_generator;
    RandomGenerator * random_generator_neighborhood;
        ;
    int initial_point[1];

    class Cell {
    public:
        int cellx, celly;
        float x1, y1, x2, y2;
        bool operator==(const Cell &o) const {
            return x1 == o.x1 && y1 == o.y1 && x2 == o.x2 && y2 == o.y2;
        }

        bool operator<(const Cell &o) const {
            if (x1 < o.x1) return true;
            if (x1 == o.x1 && y1 < o.y1) return true;
            if (x1 == o.x1 && y1 == o.y1 && x2 < o.x2) return true;
            if (x1 == o.x1 && y1 == o.y1 && x2 == o.x2 && y2 < o.y2) return true;
            else return false;
        }
    };

    std::map<int, Cell> point_2cell;
    std::map<Cell, std::vector<int>> cell_2points;
    std::vector<std::vector<Cell>> cells;

    int square_radius;
public:


    ProgressiveNapsac (const cv::Mat& input_points, unsigned int sample_size, bool reset_time) {
        assert (!input_points.empty());

        random_generator = new UniformRandomGenerator;
        random_generator_neighborhood = new UniformRandomGenerator;
        if (reset_time) random_generator->resetTime();
        random_generator->setSubsetSize(1);
        random_generator_neighborhood->setSubsetSize(sample_size);
        random_generator->resetGenerator(0, points_size-1);

        this->sample_size = sample_size;

        square_radius = 1;

        float * points = (float *) input_points.data;
        points_size = input_points.rows;

        int rows_split = 30;
        int cols_split = 30;

        /*
         *  Block are rectangles with left up and right down corners (x1,y1) (x2,y2)
            b11 b12 ... b1n
            b21 b22 ... b2n
            ...
            bm1 bm2 ... bmn
         */
        int blocks_size = rows_split*cols_split;
        cells = std::vector<std::vector<Cell>>(rows_split);

        float bx = 0, by = 0;

        // get maximum size of image
        float max_x = 0;
        float max_y = 0;
        unsigned int idx;
        for (int i = 0; i < points_size; i++) {
            idx = 2 * i;
            if (max_x < points[idx]) {
                max_x = points[idx];
            }
            if (max_y < points[idx+1]) {
                max_y = points[idx+1];
            }
        }
        // --------------------------

        float step_x = max_x / cols_split;
        float step_y = max_y / rows_split;
        Cell cell = Cell();
        for (unsigned int i = 0; i < rows_split; i++) {
            bx = 0;
            for (unsigned int j = 0; j < cols_split; j++) {
                cell.x1 = bx;
                cell.y1 = by;
                bx += step_x;
                cell.x2 = bx;
                cell.y2 = by+step_y;

                cell.cellx = i;
                cell.celly = j;
                cells[i].emplace_back(cell);
            }
             // std::cout << '\n';
            by += step_y;
        }

        for (int i = 0; i < points_size; i++) {
            idx = 2*i;
            for (int x = 0; x < rows_split; x++) {
                for (int y = 0; y < cols_split; y++) {
                    if (points[idx] >= cells[x][y].x1 && points[idx] <= cells[x][y].x2 &&
                        points[idx+1] >= cells[x][y].y1 && points[idx+1] <= cells[x][y].y2) {

                        cell_2points[cells[x][y]].push_back(i);
                        point_2cell[i] = cells[x][y];

                        // point belong only to one cell
                        break;
                    }
                }
            }
        }

        // for debug
        cv::Mat img = cv::imread ("../dataset/image1.png");
        for (int x = 0; x < rows_split; x++) {
            for (int y = 0; y < cols_split; y++) {
                cv::Scalar color = cv::Scalar (random() % 256, random() % 256, random() % 256);

                cv::rectangle (img, cv::Point_<float>(cells[x][y].x1, cells[x][y].y1),
                                 cv::Point_<float>(cells[x][y].x2, cells[x][y].y2), color, 4);

                for (int p = 0; p < cell_2points[cells[x][y]].size(); p++) {
                    cv::circle(img, cv::Point_<float>(points[2*cell_2points[cells[x][y]][p]],
                                                  points[2*cell_2points[cells[x][y]][p]+1]), 3, color, -1);
                }
            }
        }
        cv::imshow ("rect ", img);
        cv::waitKey (0);
        cv::imwrite ("../results/blocks.jpg", img);
        // -------------------------
    }

    void generateSample (int *sample) override {
        random_generator->generateUniqueRandomSet(initial_point);

        int init_pt = initial_point[0];
        Cell centered_cell = point_2cell[init_pt];
        
        int num_points_in_neighborhood = 0;
        for (int i = centered_cell.cellx-square_radius; i < centered_cell.cellx+square_radius; i++) {
            for (int j = centered_cell.celly-square_radius; j < centered_cell.celly+square_radius; j++) {
                num_points_in_neighborhood += cell_2points[cells[i][i]].size();        
            }
        }

        random_generator_neighborhood->resetGenerator(0, num_points_in_neighborhood-1);
        random_generator_neighborhood->generateUniqueRandomSet(sample);
        
        for (int i = 1; i < sample_size; i++) {
            // sample[i] = cells[][]
        }
        sample[0] = init_pt;
        
        square_radius++;
        k_iterations++;
    }


    bool isInit () override {
        return true;
    }
};



#endif //USAC_GRADUALNAPSAC_H
