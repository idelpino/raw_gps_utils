
#ifndef TYPES_RAW_GPS_UTILS_H_
#define TYPES_RAW_GPS_UTILS_H_

//includes from Eigen lib
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


//type scalar, in case working on 32-bit architectures , or benchmarking
namespace rawgpsutils
{
    //typedef float ScalarT;         // Use this for float, 32 bit precision
    typedef double ScalarT;        // Use this for double, 64 bit precision
    //typedef long double ScalarT;   // Use this for long double, 128 bit precision
}

//TODO vedi se mettere i vettori dinamici o quel che e', per ottimizzare ceres. guarda uno degli issue aperti

// Eigen namespace extension using the above defined scalar
namespace Eigen  
{
    // 1. Vectors and Matrices
    typedef Matrix<rawgpsutils::ScalarT, 2, 2, Eigen::RowMajor> Matrix2s;                ///< 2x2 matrix of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 3, 3, Eigen::RowMajor> Matrix3s;                ///< 3x3 matrix of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 4, 4, Eigen::RowMajor> Matrix4s;                ///< 4x4 matrix of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, Dynamic, Dynamic, Eigen::RowMajor> MatrixXs;    ///< variable size matrix of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 1, 1> Vector1s;                ///< 1-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 2, 1> Vector2s;                ///< 2-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 3, 1> Vector3s;                ///< 3-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 4, 1> Vector4s;                ///< 4-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, Dynamic, 1> VectorXs;          ///< variable size vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 1, 2> RowVector2s;             ///< 2-row-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 1, 3> RowVector3s;             ///< 3-row-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 1, 4> RowVector4s;             ///< 4-row-vector of real scalar_t type
    typedef Matrix<rawgpsutils::ScalarT, 1, Dynamic> RowVectorXs;       ///< variable size row-vector of real scalar_t type

    // 2. Quaternions and other rotation things
    typedef Quaternion<rawgpsutils::ScalarT> Quaternions;               ///< Quaternion of real scalar_t type
    typedef AngleAxis<rawgpsutils::ScalarT> AngleAxiss;                 ///< Angle-Axis of real scalar_t type
}
#endif
