#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <ml4kp_bridge/Matrix.h>

namespace ml4kp_bridge
{

Eigen::MatrixXd create(const ml4kp_bridge::Matrix& msg)
{
  Eigen::MatrixXd mat(msg.rows, msg.cols);
  int ij{ 0 };
  for (int i = 0; i < msg.rows; ++i)
  {
    for (int j = 0; j < msg.cols; ++j, ++ij)
    {
      mat(i, j) = msg.matrix[ij];
    }
  }
  return mat;
}

template <int Rows, int Cols>
void copy(Eigen::Matrix<double, Rows, Cols>& mat, const ml4kp_bridge::Matrix& msg)
{
  prx_assert(mat.rows() == msg.rows, "[matrix_bridge::copy] Rows don't match");
  prx_assert(mat.cols() == msg.cols, "[matrix_bridge::copy] Cols don't match");

  int ij{ 0 };
  for (int i = 0; i < msg.rows; ++i)
  {
    for (int j = 0; j < msg.cols; ++j, ++ij)
    {
      mat(i, j) = msg.matrix[ij];
    }
  }
}

template <int Rows, int Cols>
void copy(ml4kp_bridge::Matrix& msg, const Eigen::Matrix<double, Rows, Cols>& mat)
{
  msg.rows = mat.rows();
  msg.cols = mat.cols();
  msg.matrix.resize(msg.rows * msg.cols);
  int ij{ 0 };

  for (int i = 0; i < msg.rows; ++i)
  {
    for (int j = 0; j < msg.cols; ++j, ++ij)
    {
      msg.matrix[ij] = mat(i, j);
    }
  }
}
}  // namespace ml4kp_bridge