#include "DLT.h"

void DLT (const float * const points, const int * const sample, int sample_number, cv::Mat &H) {
    float x1, y1, x2, y2;
    int smpl;

    cv::Mat_<float> A (2*sample_number, 9), w, u, vt;
    float * A_ptr = (float *) A.data;

    for (int i = 0; i < sample_number; i++) {
        smpl = 4*sample[i];
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = x2*x1;
        (*A_ptr++) = x2*y1;
        (*A_ptr++) = x2;

        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = y2*x1;
        (*A_ptr++) = y2*y1;
        (*A_ptr++) = y2;
    }

    /*
     * src	decomposed matrix
     * w 	calculated singular values
     * u	calculated left singular vectors
     * vt	transposed matrix of right singular values
     *
     * Flags:
     * MODIFY_A  allow the algorithm to modify the decomposed matrix; it can
     * save space and speed up processing. currently ignored.
     *
     * NO_UV indicates that only a vector of singular values w is to be processed,
     * while u and vt will be set to empty matrices
     *
     * FULL_UV 	when the matrix is not square, by default the algorithm produces
     * u and vt matrices of sufficiently large size for the further A reconstruction;
     * if, however, FULL_UV flag is specified, u and vt will be full-size square orthogonal matrices.
     */
    // todo use eigen
    cv::SVD::compute(A, w, u, vt);

//    std::cout << "A=\n"<< A << "\n\n";
//    std::cout << "vt=\n" << vt << "\n\n";

    H = cv::Mat_<float>(vt.row(vt.rows-1).reshape (3,3));
}

bool DLT (const float * const points, int sample_number, cv::Mat &H) {
    float x1, y1, x2, y2;
    int smpl;

    cv::Mat_<float> A (2*sample_number, 9), w, u, vt;
    float * A_ptr = (float *) A.data;

    for (int i = 0; i < sample_number; i++) {
        smpl = 4*i;
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = x2*x1;
        (*A_ptr++) = x2*y1;
        (*A_ptr++) = x2;

        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = y2*x1;
        (*A_ptr++) = y2*y1;
        (*A_ptr++) = y2;
    }

    cv::SVD::compute(A, w, u, vt);

    // std::cout << "\033[1;32mSVD computed \033[0m \n";
    if (vt.empty ()) {
        std::cout << "\033[1;31mDecomposed Matrix Vt is empty\033[0m \n";
        return false;
    }

    H = cv::Mat_<float>(vt.row(vt.rows-1).reshape (3,3));

    return true;
}

