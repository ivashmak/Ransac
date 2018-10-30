#include "Math.h"

/*
 * Fast finding inverse matrix 3x3
 */
bool inverse3x3 (cv::Mat& A) {
//    assert (A.rows == 3 && A.cols == 3);

    float * A_ptr = (float *) A.data;
    float a11 = A_ptr[0];
    float a12 = A_ptr[1];
    float a13 = A_ptr[2];
    float a21 = A_ptr[3];
    float a22 = A_ptr[4];
    float a23 = A_ptr[5];
    float a31 = A_ptr[6];
    float a32 = A_ptr[7];
    float a33 = A_ptr[8];

    float detA = a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31;

    if (detA == 0) {
        std::cout << "\033[1;31mDeterminant of A is 0\033[0m\n";
        return false;
    } else if (detA < 0.0000001) {
        std::cout << "\033[1;33mDeterminant of A is very small\033[0m\n";
    }

    A_ptr[0] = (a22*a33 - a23*a32)/detA;
    A_ptr[1] = (a13*a32 - a12*a33)/detA;
    A_ptr[2] = (a12*a23 - a13*a22)/detA;
    A_ptr[3] = (a23*a31 - a21*a33)/detA;
    A_ptr[4] = (a11*a33 - a13*a31)/detA;
    A_ptr[5] = (a13*a21 - a11*a23)/detA;
    A_ptr[6] = (a21*a32 - a22*a31)/detA;
    A_ptr[7] = (a12*a31 - a11*a32)/detA;
    A_ptr[8] = (a11*a22 - a12*a21)/detA;

    return true;
}

/*
 *
 * Declare fast_pow function because c++ pow is very slow
 * https://stackoverflow.com/questions/41072787/why-is-powint-int-so-slow/41072811
 * pow(x,y) = e^(y log(x))
 * Assume that my pow is called for power at least 2.
 *
 */
float fast_pow (float n, int k) {
    float res = n * n;
    while (k > 2) {
        res *= n; k--;
    }
    return res;
}