#ifndef USAC_UTILS_H
#define USAC_UTILS_H

#include <theia/theia.h>
//#include <Eigen/Dense>

typedef Eigen::Matrix<double, 5, 3> Matrix5x3;
typedef Eigen::Matrix<double, 5, 5> Matrix5x5;

class Utils {
public:
    void getNullSpace () {
        Matrix5x3 m = Matrix5x3::Random();
        std::cout << "Here is the matrix m:" << std::endl << m << std::endl;
        Eigen::FullPivLU<Matrix5x3> lu(m);
        std::cout << "Here is, up to permutations, its LU decomposition matrix:"
             << std::endl << lu.matrixLU() << std::endl;
        std::cout << "Here is the L part:" << std::endl;
        Matrix5x5 l = Matrix5x5::Identity();
        l.block<5,3>(0,0).triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
        std::cout << l << std::endl;
        std::cout << "Here is the U part:" << std::endl;
        Matrix5x3 u = lu.matrixLU().triangularView<Eigen::Upper>();
        std::cout << u << std::endl;
        std::cout << "Let us now reconstruct the original matrix m:" << std::endl;
        std::cout << lu.permutationP().inverse() * l * u * lu.permutationQ().inverse() << std::endl;
    }
};

#endif //USAC_UTILS_H
