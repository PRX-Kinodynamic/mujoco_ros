#ifndef TORCH_NOT_BUILT
#pragma once

/**
 * @file torch_eigen_bridge.hpp
 * @author Edgar Granados
 * @brief <b> Bridge between Eigen and Torch
 * */

#include <Eigen/Dense>
#include <Eigen/Core>
#include <torch/torch.h>
#include <torch/script.h>

namespace torch_bridge
{

template <typename TorchScalar, typename Derived>
void copy_typed(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
{
  static constexpr Eigen::Index RowsComp{ Eigen::MatrixBase<Derived>::RowsAtCompileTime };
  static constexpr Eigen::Index ColsComp{ Eigen::MatrixBase<Derived>::ColsAtCompileTime };
  if (tensor.is_cuda())
  {
    // Create CPU tensor, copy data, then transfer to GPU
    torch::Tensor cpu_tensor = torch::empty_like(tensor, tensor.options().device(torch::kCPU));
    TorchScalar* data{ cpu_tensor.data_ptr<TorchScalar>() };
    Eigen::Map<Eigen::Matrix<TorchScalar, RowsComp, ColsComp>> ef(data, vec.rows(), vec.cols());
    ef = vec.template cast<TorchScalar>();
    tensor.copy_(cpu_tensor);
  }
  else
  {
    TorchScalar* data{ tensor.data_ptr<TorchScalar>() };
    Eigen::Map<Eigen::Matrix<TorchScalar, RowsComp, ColsComp>> ef(data, vec.rows(), vec.cols());
    ef = vec.template cast<TorchScalar>();
  }
}

template <typename TorchScalar, typename Derived>
void copy_typed(Eigen::MatrixBase<Derived> const& vec, const torch::Tensor& tensor)
{
  using RowMajorMat = Eigen::Matrix<TorchScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  // Move tensor to CPU if on CUDA before accessing data pointer
  torch::Tensor cpu_tensor = tensor.is_cuda() ? tensor.to(torch::kCPU) : tensor;
  auto sizes = cpu_tensor.sizes();
  const int rows{ sizes.size() > 0 ? static_cast<int>(sizes[0]) : 0 };
  const int cols{ sizes.size() > 1 ? static_cast<int>(sizes[1]) : 1 };
  TorchScalar* data{ cpu_tensor.data_ptr<TorchScalar>() };
  Eigen::Map<RowMajorMat> ef(data, rows, cols);
  const_cast<Eigen::MatrixBase<Derived>&>(vec) = ef.template cast<typename Derived::Scalar>();
}

template <typename Derived>
void copy(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
{
  static constexpr Eigen::Index RowsComp{ Eigen::MatrixBase<Derived>::RowsAtCompileTime };
  static constexpr Eigen::Index ColsComp{ Eigen::MatrixBase<Derived>::ColsAtCompileTime };
  if (tensor.is_cuda())
  {
    // Create CPU tensor, copy data, then transfer to GPU
    torch::Tensor cpu_tensor = torch::empty_like(tensor, tensor.options().device(torch::kCPU));
    double* data{ cpu_tensor.data_ptr<double>() };
    Eigen::Map<Eigen::Matrix<double, RowsComp, ColsComp>> ef(data, vec.rows(), vec.cols());
    ef = vec.template cast<double>();
    tensor.copy_(cpu_tensor);
  }
  else
  {
    double* data{ tensor.data_ptr<double>() };
    Eigen::Map<Eigen::Matrix<double, RowsComp, ColsComp>> ef(data, vec.rows(), vec.cols());
    ef = vec.template cast<double>();
  }
}

template <typename Derived>
void copy(Eigen::MatrixBase<Derived> const& vec, const torch::Tensor& tensor)
{
  using RowMajorMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  // Move tensor to CPU if on CUDA before accessing data pointer
  torch::Tensor cpu_tensor = tensor.is_cuda() ? tensor.to(torch::kCPU) : tensor;
  auto sizes = cpu_tensor.sizes();
  const int rows{ sizes.size() > 0 ? static_cast<int>(sizes[0]) : 0 };
  const int cols{ sizes.size() > 1 ? static_cast<int>(sizes[1]) : 1 };
  double* data{ cpu_tensor.data_ptr<double>() };
  Eigen::Map<RowMajorMat> ef(data, rows, cols);
  const_cast<Eigen::MatrixBase<Derived>&>(vec) = ef.template cast<double>();
}

template <typename Derived>
void copy_f32(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
{
  copy_typed<float>(tensor, vec);
}

template <typename Derived>
void copy_f32(Eigen::MatrixBase<Derived> const& vec, const torch::Tensor& tensor)
{
  copy_typed<float>(vec, tensor);
}

template <typename Derived>
void copy_f64(torch::Tensor& tensor, const Eigen::MatrixBase<Derived>& vec)
{
  copy_typed<double>(tensor, vec);
}

template <typename Derived>
void copy_f64(Eigen::MatrixBase<Derived> const& vec, const torch::Tensor& tensor)
{
  copy_typed<double>(vec, tensor);
}

inline torch::Tensor make_tensor_f32(int rows, int cols = 1, torch::Device device = torch::kCPU)
{
  return torch::empty({rows, cols}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
}

inline torch::Tensor make_tensor_f64(int rows, int cols = 1, torch::Device device = torch::kCPU)
{
  return torch::empty({rows, cols}, torch::TensorOptions().dtype(torch::kFloat64).device(device));
}

inline torch::Tensor make_vector_f32(int size, torch::Device device = torch::kCPU)
{
  return torch::empty({size}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
}

inline torch::Tensor make_vector_f64(int size, torch::Device device = torch::kCPU)
{
  return torch::empty({size}, torch::TensorOptions().dtype(torch::kFloat64).device(device));
}

}  // namespace torch_bridge
#endif