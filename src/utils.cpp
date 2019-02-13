// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "utils.hpp"
#include "nearest_neighbors.hpp"

void densitySort (const cv::Mat &points, int knn, cv::Mat &sorted_points) {
    // get neighbors
    cv::Mat neighbors, neighbors_dists;
    NearestNeighbors::getNearestNeighbors_nanoflann(points, knn, neighbors, true, neighbors_dists);
    //

    std::vector<int> sorted_idx(points.rows);
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
    float sum1, sum2;
    int idxa, idxb;
    float *neighbors_dists_ptr = (float *) neighbors_dists.data;
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&](int a, int b) {
        sum1 = 0, sum2 = 0;
        idxa = knn * a, idxb = knn * b;
        for (int i = 0; i < knn; i++) {
            sum1 += neighbors_dists_ptr[idxa + i];
            sum2 += neighbors_dists_ptr[idxb + i];
        }
        return sum1 < sum2;
    });

    for (int i = 0; i < points.rows; i++) {
        sorted_points.push_back(points.row(sorted_idx[i]));
    }
}

/*
 * filename = path/name.ext
 * path
 * name
 * ext
 */
void splitFilename (const std::string &filename, std::string &path, std::string &name, std::string &ext) {
    const unsigned long dot = filename.find_last_of('.');
    const unsigned long slash = filename.find_last_of('/');
    // substr (pos, n) take substring of size n starting from position pos.
    path = filename.substr(0, slash+1);
    name = filename.substr(slash+1, dot-slash-1);
    ext = filename.substr(dot+1, filename.length()-1);
}

void swap(int* a, int* b) {
    int t = *a;
    *a = *b;
    *b = t;
}

int quicksort_median (int * array, unsigned int k_minth, unsigned int left, unsigned int right) {
    unsigned int lenght = right - left;
    if (lenght == 0) {
        return array[left];
    }
    int pivot_val = array[right];
    int j;
    int right_ = right-1;
    unsigned int values_less_eq = 1;
    for (j = left; j <= right_;) {
        if (array[j] <= pivot_val) {
            j++;
            values_less_eq++;
        } else {
            swap(&array[j], &array[right_]);
            right_--;
        }
    }
    if (values_less_eq == k_minth) return pivot_val;
    if (k_minth > values_less_eq) {
        quicksort_median(array, k_minth - values_less_eq, j, right-1);
    } else {
        if (j == left) j++;
        quicksort_median(array, k_minth, left, j-1);
    }
}

// find median using quicksort with average O(n) complexity. Worst case is O(n^2).
int findMedian (int * array, unsigned int length, bool make_copy) {
    if (make_copy) {
        int * arr = new int [length];
        std::copy (array, array+length, arr);
        if (length % 2 == 1) {
            // odd number of values
            return quicksort_median (arr, length/2+1, 0, length-1);
        } else {
            // even: return average
            return (quicksort_median(arr, length/2, 0, length-1) + quicksort_median(array, length/2+1, 0, length-1))/2;
        }
    } else {
        if (length % 2 == 1) {
            return quicksort_median (array, length/2+1, 0, length-1);
        } else {
            return (quicksort_median(array, length/2, 0, length-1) + quicksort_median(array, length/2+1, 0, length-1))/2;
        }

    }
}
