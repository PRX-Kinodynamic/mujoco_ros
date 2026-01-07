#include <Eigen/Dense>
#include <Eigen/Core>
#include <cstddef>
#include <iterator>
#include "Eigen/src/Core/Matrix.h"
#include <torch_bridge/TorchQuery.h>

namespace torch_bridge
{

template <typename Vector>
void update_request(torch_bridge::TorchQuery& query, const Vector& vector)
{
  for (auto& ei : vector)
  {
    query.request.data.push_back(ei);
  }
}

template <typename Car, typename... Cdr, std::enable_if_t<(sizeof...(Cdr) > 0), bool> = true>
void update_request(torch_bridge::TorchQuery& query, const Car& car, const Cdr&... cdr)
{
  update_request(query, car);
  update_request(query, cdr...);
}

template <int Dim>
inline void get_result(std::vector<double>& data, Eigen::Vector<double, Dim>& vector)
{
  std::size_t idx{ 0 };

  for (auto& ei : vector)
  {
    ei = data[idx];
    idx++;
  }
  // data.insert(query.response.result.end(),data.begin(),
  //                              data.begin() + idx);
  data.erase(data.begin(), data.begin() + idx);
}

template <typename Car>
inline void get_result(torch_bridge::TorchQuery& query, Car& car)
{
  get_result(query.response.result, car);
}

template <typename Car, typename... Cdr, std::enable_if_t<(sizeof...(Cdr) > 0), bool> = true>
inline void get_result(torch_bridge::TorchQuery& query, Car& car, Cdr&... cdr)
{
  get_result(query, car);
  get_result(query, cdr...);
}
template <int Rows, int Cols>
void get_jacobian(torch_bridge::TorchQuery& query, Eigen::Matrix<double, Rows, Cols>& mat)
{
  std::size_t idx{ 0 };
  for (int i = 0; i < mat.rows(); ++i)
  {
    for (int j = 0; j < mat.cols(); ++j)
    {
      mat(i, j) = query.response.jacobians[idx];
      idx++;
    }
  }
  query.response.jacobians.erase(query.response.jacobians.begin(), query.response.jacobians.begin() + idx);

  // get_result(query.response.jacobians, mat.row(i).transpose());
}

template <typename Car, typename... Cdr, std::enable_if_t<(sizeof...(Cdr) > 0), bool> = true>
void get_jacobian(torch_bridge::TorchQuery& query, Car& car, Cdr&... cdr)
{
  get_jacobian(query, car);
  get_jacobian(query, cdr...);
}

}  // namespace torch_bridge