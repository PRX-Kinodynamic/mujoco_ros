#ifndef TORCH_NOT_BUILT
#pragma once

/**
 * @file gpu_plant.hpp
 * @brief GPU implementation of MuSHR plant dynamics using PyTorch tensor operations.
 *
 * This keeps all computation on GPU, avoiding CPU-GPU round trips during inference.
 * The analytical C++ implementation in mushr_factors.hpp can be used as a fallback.
 */

#include <torch/torch.h>

namespace torch_bridge
{

/**
 * @brief GPU-based MuSHR plant dynamics.
 *
 * Implements the same dynamics as prx_models::mushr_CtrlAccel_t::predict() but using
 * PyTorch tensor operations that run entirely on GPU.
 *
 * Key equations (Ackermann steering model):
 *   1. delta = poly(deltaIn)           - steering polynomial
 *   2. beta = atan(tan(delta)/2)       - slip angle from steering
 *   3. beta_prev = atan2(vy, vx)       - previous slip angle
 *   4. qd0 = Adj(inv(T_beta_prev)) * xd0  - transform to body frame
 *   5. xd1_zero = qd0 + [AccIn, 0, 0] * dt  - Euler integration
 *   6. xd1_adj = Adj(T_beta) * xd1_zero    - transform back
 *   7. w_new = [0, 0, (omega*Vcurr - omega_prev*Vprev) * friction]  - friction correction
 *   8. xd1 = xd1_adj + w_new + residual
 */
class GpuPlant
{
public:
  /**
   * @brief Construct GPU plant with pre-allocated tensors.
   *
   * @param L Wheelbase length
   * @param steering_poly Steering polynomial coefficients [c0, c1, c2, c3] for c0*x^3 + c1*x^2 + c2*x + c3
   * @param dtype Tensor dtype (torch::kFloat32 or torch::kFloat64)
   * @param device Target device (should be CUDA)
   */
  GpuPlant(double L, const std::vector<double>& steering_poly, torch::Dtype dtype, torch::Device device)
      : L_(L), dtype_(dtype), device_(device)
  {
    // Store constants as scalar tensors on device
    L_tensor_ = torch::tensor(L, torch::TensorOptions().dtype(dtype).device(device));
    two_ = torch::tensor(2.0, torch::TensorOptions().dtype(dtype).device(device));
    half_ = torch::tensor(0.5, torch::TensorOptions().dtype(dtype).device(device));
    eps_ = torch::tensor(1e-8, torch::TensorOptions().dtype(dtype).device(device));

    // Steering polynomial coefficients
    poly_ = torch::tensor(steering_poly, torch::TensorOptions().dtype(dtype).device(device));
  }

  /**
   * @brief Compute plant dynamics on GPU.
   *
   * All inputs and outputs are GPU tensors. No CPU synchronization occurs.
   *
   * @param xd0 Input velocity state [vx, vy, w], shape (3,), on GPU
   * @param u_eff Effective control [acc, delta], shape (2,), on GPU
   * @param friction Friction coefficient, scalar tensor on GPU
   * @param accel_gain Acceleration gain parameter, scalar tensor on GPU
   * @param dt Timestep, scalar tensor on GPU
   * @param residual Residual correction [dvx, dvy, dw], shape (3,), on GPU
   * @return xd1 Output velocity state [vx, vy, w], shape (3,), on GPU
   */
  torch::Tensor forward(const torch::Tensor& xd0, const torch::Tensor& u_eff, const torch::Tensor& friction,
                        const torch::Tensor& accel_gain, const torch::Tensor& dt,
                        const torch::Tensor& residual) const
  {
    // Extract state components
    torch::Tensor vx = xd0[0];
    torch::Tensor vy = xd0[1];
    torch::Tensor w = xd0[2];

    // Extract control components
    torch::Tensor u_acc = u_eff[0];
    torch::Tensor delta_in = u_eff[1];

    // Evaluate steering polynomial: delta = c0*x^3 + c1*x^2 + c2*x + c3
    torch::Tensor delta = poly_[0] * delta_in.pow(3) + poly_[1] * delta_in.pow(2) + poly_[2] * delta_in + poly_[3];

    // Compute slip angles
    // beta = atan(tan(delta) / 2)  -- bicycle model approximation
    torch::Tensor beta = torch::atan(torch::tan(delta) * half_);

    // beta_prev = atan2(vy, vx)
    torch::Tensor beta_prev = torch::atan2(vy, vx + eps_);

    // Angular velocities: omega = 2 * sin(beta) / L
    torch::Tensor omega = two_ * torch::sin(beta) / L_tensor_;
    torch::Tensor omega_prev = two_ * torch::sin(beta_prev) / L_tensor_;

    // Velocity magnitudes (with sign from acceleration command)
    torch::Tensor v_prev_mag = torch::sqrt(vx * vx + vy * vy + eps_);
    torch::Tensor v_prev = torch::copysign(v_prev_mag, u_acc);

    // Acceleration
    torch::Tensor acc_in = u_acc * accel_gain;

    // Transform to body frame: qd0 = Adj(inv(T_beta_prev)) * xd0
    // Adj(inv(0,0,theta)) = Adj(0,0,-theta)
    // For velocity: [cos(-theta)*vx - sin(-theta)*vy, sin(-theta)*vx + cos(-theta)*vy, w]
    //             = [cos(theta)*vx + sin(theta)*vy, -sin(theta)*vx + cos(theta)*vy, w]
    torch::Tensor cos_bp = torch::cos(beta_prev);
    torch::Tensor sin_bp = torch::sin(beta_prev);
    torch::Tensor sign_acc = torch::sign(u_acc);

    torch::Tensor qd0_x = sign_acc * (cos_bp * vx + sin_bp * vy);
    torch::Tensor qd0_y = sign_acc * (-sin_bp * vx + cos_bp * vy);
    torch::Tensor qd0_w = sign_acc * w;

    // Euler integration: xd1_zero = qd0 + [acc_in, 0, 0] * dt
    torch::Tensor xd1z_x = qd0_x + acc_in * dt;
    torch::Tensor xd1z_y = qd0_y;
    torch::Tensor xd1z_w = qd0_w;

    // Current velocity magnitude
    torch::Tensor v_curr_mag = torch::sqrt(xd1z_x * xd1z_x + xd1z_y * xd1z_y + eps_);
    torch::Tensor v_curr = torch::copysign(v_curr_mag, u_acc);

    // Transform back: xd1_adj = Adj(T_beta) * xd1_zero
    // Adj(0,0,theta) * [vx, vy, w] = [cos(theta)*vx - sin(theta)*vy, sin(theta)*vx + cos(theta)*vy, w]
    torch::Tensor cos_b = torch::cos(beta);
    torch::Tensor sin_b = torch::sin(beta);

    torch::Tensor xd1a_x = cos_b * xd1z_x - sin_b * xd1z_y;
    torch::Tensor xd1a_y = sin_b * xd1z_x + cos_b * xd1z_y;
    torch::Tensor xd1a_w = xd1z_w;

    // Friction correction: w_new = [0, 0, (omega*Vcurr - omega_prev*Vprev) * friction]
    torch::Tensor thd_curr = omega * v_curr;
    torch::Tensor thd_prev = omega_prev * v_prev;
    torch::Tensor w_correction = (thd_curr - thd_prev) * friction;

    // Final result: xd1 = xd1_adj + w_new + residual
    torch::Tensor xd1 = torch::stack({xd1a_x + residual[0], xd1a_y + residual[1], xd1a_w + w_correction + residual[2]});

    return xd1;
  }

private:
  double L_;
  torch::Dtype dtype_;
  torch::Device device_;

  // Pre-allocated constant tensors
  torch::Tensor L_tensor_;
  torch::Tensor two_;
  torch::Tensor half_;
  torch::Tensor eps_;
  torch::Tensor poly_;
};

}  // namespace torch_bridge

#endif  // TORCH_NOT_BUILT
