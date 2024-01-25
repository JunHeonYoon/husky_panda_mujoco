#pragma once
#include <Eigen/Dense>
#include <string>

using namespace Eigen;

typedef const Eigen::Ref<const Eigen::MatrixXd>   & Cref_matrixXd;
typedef const Eigen::Ref<const Eigen::VectorXd>   & Cref_vectorXd;
typedef const Eigen::Ref<const Eigen::Matrix2d>   & Cref_matrix2d;

typedef Eigen::Ref<Eigen::MatrixXd>   Ref_matrixXd;
typedef Eigen::Ref<Eigen::VectorXd>   Ref_vectorXd;
typedef Eigen::Ref<Eigen::Matrix2d>   Ref_matrix2d;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRowMajor;
typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3Xd;

typedef std::size_t Index;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6x;

#define panda_dof 7 // DOF of panda
#define ee_dof 6 // DOF of End-Effector
#define panda_num_links 9 // link0~7, hand