#ifndef USAC_FUNDAMENTALMATRIXESTIMATION_H
#define USAC_FUNDAMENTALMATRIXESTIMATION_H

// 7,8-point algortihm
// R. I. Hartley and  A. Zisserman, Multiple  View  Geometry  in  Computer Vision. Cambridge University Press, 2
// http://cvrs.whu.edu.cn/downloads/ebooks/Multiple%20View%20Geometry%20in%20Computer%20Vision%20(Second%20Edition).pdf
// page 279

#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>
#include <iostream>

class FundamentalMatrixEstimation {
protected:

public:
    int sevenPointsAlg(cv::InputArray  input_points1, cv::InputArray input_points2, cv::Mat &F, double focal = 1.0,
                        cv::Point2d pp = cv::Point2d(0, 0)) {

        cv::Mat mask;
        int result = run(input_points1, input_points2, F, mask);
        std::cout << "RESULT = " << result << '\n';
        return result;
    }

    int runKernel( cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model) const {
        cv::Mat m1 = _m1.getMat(), m2 = _m2.getMat();
        cv::Mat_<double> F(9, 3);
        int n = run7Point(m1, m2, F) ;

        if (n == 0)
            _model.release();
        else
            F.rowRange(0, n*3).copyTo(_model);

        return n;
    }


    int run7Point (cv::Mat& _m1, cv::Mat& _m2, cv::Mat& _fmatrix) const {
        double a[7*9], w[7], u[9*9], v[9*9], c[4], r[3];
        double* f1, *f2;
        double t0, t1, t2;
        cv::Mat_<double> A( 7, 9, a);
        cv::Mat_<double> U( 7, 9, u);
        cv::Mat_<double> Vt( 9, 9, v);
        cv::Mat_<double> W( 7, 1, w);
        cv::Mat_<double> coeffs( 1, 4, c);
        cv::Mat_<double> roots( 1, 3, r);
        const cv::Point2f* m1 = _m1.ptr<cv::Point2f>();
        const cv::Point2f* m2 = _m2.ptr<cv::Point2f>();
        double* fmatrix = _fmatrix.ptr<double>();
        int i, k, n;

        // form a linear system: i-th row of A(=a) represents
        // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0

        for( i = 0; i < 7; i++ )
        {
            double x0 = m1[i].x, y0 = m1[i].y;
            double x1 = m2[i].x, y1 = m2[i].y;

            a[i*9+0] = x1*x0;
            a[i*9+1] = x1*y0;
            a[i*9+2] = x1;
            a[i*9+3] = y1*x0;
            a[i*9+4] = y1*y0;
            a[i*9+5] = y1;
            a[i*9+6] = x0;
            a[i*9+7] = y0;
            a[i*9+8] = 1;
        }

        // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
        // the solution is linear subspace of dimensionality 2.
        // => use the last two singular vectors as a basis of the space
        // (according to SVD properties)
        cv::SVDecomp( A, W, U, Vt, cv::SVD::MODIFY_A + cv::SVD::FULL_UV );
        f1 = v + 7*9;
        f2 = v + 8*9;

        // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary f. matrix.
        // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
        // so f ~ lambda*f1 + (1 - lambda)*f2.
        // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to find lambda.
        // it will be a cubic equation.
        // find c - polynomial coefficients.
        for( i = 0; i < 9; i++ )
            f1[i] -= f2[i];

        t0 = f2[4]*f2[8] - f2[5]*f2[7];
        t1 = f2[3]*f2[8] - f2[5]*f2[6];
        t2 = f2[3]*f2[7] - f2[4]*f2[6];

        c[3] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2;

        c[2] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2 -
               f1[3]*(f2[1]*f2[8] - f2[2]*f2[7]) +
               f1[4]*(f2[0]*f2[8] - f2[2]*f2[6]) -
               f1[5]*(f2[0]*f2[7] - f2[1]*f2[6]) +
               f1[6]*(f2[1]*f2[5] - f2[2]*f2[4]) -
               f1[7]*(f2[0]*f2[5] - f2[2]*f2[3]) +
               f1[8]*(f2[0]*f2[4] - f2[1]*f2[3]);

        t0 = f1[4]*f1[8] - f1[5]*f1[7];
        t1 = f1[3]*f1[8] - f1[5]*f1[6];
        t2 = f1[3]*f1[7] - f1[4]*f1[6];

        c[1] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2 -
               f2[3]*(f1[1]*f1[8] - f1[2]*f1[7]) +
               f2[4]*(f1[0]*f1[8] - f1[2]*f1[6]) -
               f2[5]*(f1[0]*f1[7] - f1[1]*f1[6]) +
               f2[6]*(f1[1]*f1[5] - f1[2]*f1[4]) -
               f2[7]*(f1[0]*f1[5] - f1[2]*f1[3]) +
               f2[8]*(f1[0]*f1[4] - f1[1]*f1[3]);

        c[0] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2;

        // solve the cubic equation; there can be 1 to 3 roots ...
        n = cv::solveCubic( coeffs, roots );

        if( n < 1 || n > 3 )
            return n;

        for( k = 0; k < n; k++, fmatrix += 9 )
        {
            // for each root form the fundamental matrix
            double lambda = r[k], mu = 1.;
            double s = f1[8]*r[k] + f2[8];


            // normalize each matrix, so that F(3,3) (~fmatrix[8]) == 1
            if( fabs(s) > DBL_EPSILON )
            {
                mu = 1./s;
                lambda *= mu;
                fmatrix[8] = 1.;
            }
            else
                fmatrix[8] = 0.;

            for( i = 0; i < 8; i++ )
                fmatrix[i] = f1[i]*lambda + f2[i]*mu;
        }

        return n;
    }
    double confidence = 0.99;
    int modelPoints = 7;
    int maxIters = 1000;
    bool checkPartialSubsets = true;

    bool run(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model, cv::OutputArray _mask) const {
        const double outlierRatio = 0.45;
        bool result = false;
        cv::Mat m1 = _m1.getMat(), m2 = _m2.getMat();
        cv::Mat ms1, ms2, err, errf, mask, mask0, bestModel, model;

        int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
        int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
        int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
        double minMedian = DBL_MAX;

        cv::RNG rng((uint64)-1);

        CV_Assert( count >= 0 && count2 == count );
        if( count < modelPoints )
            return false;

        if( _mask.needed() )
        {
            std::cout << "mask needed\n";
            _mask.create(count, 1, CV_8U, -1, true);
            mask0 = mask = _mask.getMat();
            CV_Assert( (mask.cols == 1 || mask.rows == 1) && (int)mask.total() == count );
        }

        if( count == modelPoints )
        {
            if( runKernel(m1, m2, bestModel) <= 0 )
                return false;
            bestModel.copyTo(_model);
            mask.setTo(cv::Scalar::all(1));
            return true;
        }

        int iter, niters = RANSACUpdateNumIters(confidence, outlierRatio, modelPoints, maxIters);
        niters = MAX(niters, 3);

        for( iter = 0; iter < niters; iter++ )
        {
            int i, nmodels;
            if( count > modelPoints )
            {
                bool found = getSubset( m1, m2, ms1, ms2, rng );
                if( !found )
                {
                    if( iter == 0 )
                        return false;
                    break;
                }
            }

            nmodels = runKernel( ms1, ms2, model );
            if( nmodels <= 0 ) {
                std::cout << "no roots\n";
                continue;
            }
            CV_Assert( model.rows % nmodels == 0 );
            cv::Size modelSize(model.cols, model.rows/nmodels);

            for( i = 0; i < nmodels; i++ )
            {
                cv::Mat model_i = model.rowRange( i*modelSize.height, (i+1)*modelSize.height );
                computeError( m1, m2, model_i, err );
                if( err.depth() != CV_32F )
                    err.convertTo(errf, CV_32F);
                else
                    errf = err;
                CV_Assert( errf.isContinuous() && errf.type() == CV_32F && (int)errf.total() == count );
                std::nth_element(errf.ptr<int>(), errf.ptr<int>() + count/2, errf.ptr<int>() + count);
                double median = errf.at<float>(count/2);

                if( median < minMedian )
                {
                    minMedian = median;
                    model_i.copyTo(bestModel);
                }
            }
        }

        if( minMedian < DBL_MAX )
        {
            double sigma = 2.5*1.4826*(1 + 5./(count - modelPoints))*std::sqrt(minMedian);
            sigma = MAX( sigma, 0.001 );

            count = findInliers( m1, m2, bestModel, err, mask, sigma );
            if( _mask.needed() && mask0.data != mask.data )
            {
                if( mask0.size() == mask.size() )
                    mask.copyTo(mask0);
                else
                    cv::transpose(mask, mask0);
            }
            bestModel.copyTo(_model);
            result = count >= modelPoints;
        }
        else
            _model.release();

        return result;
    }


    int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters ) const {

        p = MAX(p, 0.);
        p = MIN(p, 1.);
        ep = MAX(ep, 0.);
        ep = MIN(ep, 1.);

        // avoid inf's & nan's
        double num = MAX(1. - p, DBL_MIN);
        double denom = 1. - std::pow(1. - ep, modelPoints);
        if( denom < DBL_MIN )
            return 0;

        num = std::log(num);
        denom = std::log(denom);

        return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
    }

    bool getSubset( const cv::Mat& m1, const cv::Mat& m2,
                    cv::Mat& ms1, cv::Mat& ms2, cv::RNG& rng,
                    int maxAttempts=1000 ) const {
        cv::AutoBuffer<int> _idx(modelPoints);
        int* idx = _idx;
        int i = 0, j, k, iters = 0;
        int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
        int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
        int esz1 = (int)m1.elemSize1()*d1, esz2 = (int)m2.elemSize1()*d2;
        int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
        const int *m1ptr = m1.ptr<int>(), *m2ptr = m2.ptr<int>();

        ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
        ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

        int *ms1ptr = ms1.ptr<int>(), *ms2ptr = ms2.ptr<int>();

        CV_Assert( count >= modelPoints && count == count2 );
        CV_Assert( (esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0 );
        esz1 /= sizeof(int);
        esz2 /= sizeof(int);

        for(; iters < maxAttempts; iters++)
        {
            for( i = 0; i < modelPoints && iters < maxAttempts; )
            {
                int idx_i = 0;
                for(;;)
                {
                    idx_i = idx[i] = rng.uniform(0, count);
                    for( j = 0; j < i; j++ )
                        if( idx_i == idx[j] )
                            break;
                    if( j == i )
                        break;
                }
                for( k = 0; k < esz1; k++ )
                    ms1ptr[i*esz1 + k] = m1ptr[idx_i*esz1 + k];
                for( k = 0; k < esz2; k++ )
                    ms2ptr[i*esz2 + k] = m2ptr[idx_i*esz2 + k];
                if( checkPartialSubsets && !checkSubset( ms1, ms2, i+1 ))
                {
                    // we may have selected some bad points;
                    // so, let's remove some of them randomly
                    i = rng.uniform(0, i+1);
                    iters++;
                    continue;
                }
                i++;
            }
            if( !checkPartialSubsets && i == modelPoints && !checkSubset(ms1, ms2, i))
                continue;
            break;
        }

        return i == modelPoints && iters < maxAttempts;
    }

    bool checkSubset( cv::InputArray _ms1, cv::InputArray _ms2, int count ) const {
        cv::Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();
        return !haveCollinearPoints(ms1, count) && !haveCollinearPoints(ms2, count);
    }

    int findInliers( const cv::Mat& m1, const cv::Mat& m2, const cv::Mat& model, cv::Mat& err, cv::Mat& mask, double thresh ) const
    {
        computeError( m1, m2, model, err );
        mask.create(err.size(), CV_8U);

        CV_Assert( err.isContinuous() && err.type() == CV_32F && mask.isContinuous() && mask.type() == CV_8U);
        const float* errptr = err.ptr<float>();
        uchar* maskptr = mask.ptr<uchar>();
        float t = (float)(thresh*thresh);
        int i, n = (int)err.total(), nz = 0;
        for( i = 0; i < n; i++ )
        {
            int f = errptr[i] <= t;
            maskptr[i] = (uchar)f;
            nz += f;
        }
        return nz;
    }

    void computeError( cv::InputArray _m1, cv::InputArray _m2, cv::InputArray _model, cv::OutputArray _err ) const  {
        cv::Mat __m1 = _m1.getMat(), __m2 = _m2.getMat(), __model = _model.getMat();
        int i, count = __m1.checkVector(2);
        const cv::Point2f* m1 = __m1.ptr<cv::Point2f>();
        const cv::Point2f* m2 = __m2.ptr<cv::Point2f>();
        const double* F = __model.ptr<double>();
        _err.create(count, 1, CV_32F);
        float* err = _err.getMat().ptr<float>();

        for( i = 0; i < count; i++ ) {
            double a, b, c, d1, d2, s1, s2;

            a = F[0]*m1[i].x + F[1]*m1[i].y + F[2];
            b = F[3]*m1[i].x + F[4]*m1[i].y + F[5];
            c = F[6]*m1[i].x + F[7]*m1[i].y + F[8];

            s2 = 1./(a*a + b*b);
            d2 = m2[i].x*a + m2[i].y*b + c;

            a = F[0]*m2[i].x + F[3]*m2[i].y + F[6];
            b = F[1]*m2[i].x + F[4]*m2[i].y + F[7];
            c = F[2]*m2[i].x + F[5]*m2[i].y + F[8];

            s1 = 1./(a*a + b*b);
            d1 = m1[i].x*a + m1[i].y*b + c;

            err[i] = (float)std::max(d1*d1*s1, d2*d2*s2);
        }
    }


    inline bool haveCollinearPoints( const cv::Mat& m, int count ) const {
        int j, k, i = count-1;
        const cv::Point2f* ptr = m.ptr<cv::Point2f>();

        // check that the i-th selected point does not belong
        // to a line connecting some previously selected points
        // also checks that points are not too close to each other
        for( j = 0; j < i; j++ )
        {
            double dx1 = ptr[j].x - ptr[i].x;
            double dy1 = ptr[j].y - ptr[i].y;
            for( k = 0; k < j; k++ )
            {
                double dx2 = ptr[k].x - ptr[i].x;
                double dy2 = ptr[k].y - ptr[i].y;
                if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
                    return true;
            }
        }
        return false;
    }

    void sevenPointsOpencv (cv::InputArray input_points1, cv::InputArray input_points2, cv::Mat &F) {
        F = cv::findFundamentalMat(input_points1, input_points2, CV_FM_7POINT);
//        Fundamental Matrix OpenCV =
//        [3.092595693820338e-07, 3.960475423740424e-06, -0.00318188713681572;
//        -1.4019671798225e-06, 1.085673745502701e-06, 0.003750889880367869;
//        0.00067657540236854, -0.005536718913293548, 1]
    }

};

#endif //USAC_FUNDAMENTALMATRIXESTIMATION_H
