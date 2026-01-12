# Torch Bridge - Sysid Model Integration

Integration of `mushr_mujoco_sysid` learned dynamics models into ROS via the `TorchQuery` service.

## Overview

This integration allows you to serve any trained sysid model via ROS for real-time dynamics prediction. The server:
- Loads models from experiment directories (config.json, best.pt, standardizers.json)
- Supports all model types (structured, direct) and configurations (control adapters, friction learning, residuals)
- Provides fast value-only inference and optional jacobian computation
- Uses pre-allocated buffers and optional JIT/compile for performance
- Is launched via `torch_service_server_wrapper.sh` to ensure the intended Python environment is used

## Quick Start

### 1. Run the smoke test (no ROS required)

Test your model loading and inference before integrating with ROS:

```bash
cd /common/home/st1122/Projects/ros_workspace/src/infrastructure/torch_bridge/scripts

# Basic test
python test_sysid_adapter.py \
  --exp_dir /path/to/your/experiment \
  --dt 0.05

# With performance options
python test_sysid_adapter.py \
  --exp_dir /path/to/your/experiment \
  --dt 0.05 \
  --device cuda \
  --use_compile \
  --num_calls 1000
```

The test will:
- Load the model and verify all artifacts are present
- Run inference timing tests
- Compute and verify jacobians (analytical vs numerical)
- Perform a short rollout to check stability

### 2. Launch the ROS service

#### Option A: Launch with the best model (v3B)

```bash
roslaunch torch_bridge torch_service_v3B.launch
```

#### Option B: Launch with a custom experiment

```bash
roslaunch torch_bridge torch_service_sysid.launch \
  exp_dir:=/path/to/your/experiment \
  dt:=0.05 \
  device:=cpu
```

#### Option C: Launch with performance options

```bash
roslaunch torch_bridge torch_service_v3B.launch \
  device:=cuda \
  use_compile:=true
```

### 3. Test the service with the C++ timing node

```bash
# In another terminal
rosrun torch_bridge torch_service_timing _total_calls:=1000
```

This will call the service 1000 times with and without jacobians and report timing statistics.

## Architecture

```
┌─────────────────────────────────────────────┐
│  C++ Client (planner/controller)            │
│  - Uses query_utils.hpp helpers             │
└─────────────────┬───────────────────────────┘
                  │ TorchQuery request
                  │ (inputs=2, dims=[3,2])
                  │ data=[vx,vy,w, vel_cmd,steer_cmd]
                  ▼
┌─────────────────────────────────────────────┐
│  torch_service_server.py                    │
│  - Parses request                           │
│  - Calls SysidModelAdapter                  │
│  - Packs response + optional jacobians      │
└─────────────────┬───────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────┐
│  SysidModelAdapter                          │
│  - Loads config, checkpoint, standardizers  │
│  - Normalizes inputs / denormalizes outputs │
│  - Fast inference with torch.inference_mode │
│  - Optional JIT/compile for value path      │
│  - Jacobian computation on demand           │
└─────────────────┬───────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────┐
│  mushr_mujoco_sysid                         │
│  - build_model() from config                │
│  - StructuredDynamicsModel / DirectDynamicsModel │
│  - ControlAdapter, friction, residuals      │
└─────────────────────────────────────────────┘
```

## Service Interface

### Request (TorchQuery)
```
int16 inputs = 2                  # Two input tensors
int16[] input_dimensions = [3, 2] # xd0 is 3D, ut is 2D
float64[] data = [vx, vy, w, vel_cmd, steer_cmd]
bool compute_jacobians             # Whether to compute jacobians
```

### Response (TorchQueryResponse)
```
int16 outputs = 1                  # One output tensor
int16[] output_dimensions = [3]    # xd_next is 3D
float64[] result = [vx_next, vy_next, w_next]
float64[] jacobians = [...]        # If compute_jacobians=true:
                                   # - First 9 elements: d(xd_next)/d(xd0) row-major (3x3)
                                   # - Next 6 elements: d(xd_next)/d(ut) row-major (3x2)
```

## ROS Parameters

All parameters are namespaced under the node (`~`):

- **`~exp_dir`** (required): Path to experiment directory
  - Must contain: `config.json`, `best.pt`, `standardizers.json`
  - Example: `/path/to/experiments/v3B_struct_h10_tf0_seed4/v3B_struct_h10_tf0_seed4`

- **`~service_name`** (default: `/torch/service`): ROS service name

- **`~dt`** (default: `0.05`): Fixed timestep for dynamics forward pass
  - Should match the training data timestep

- **`~device`** (default: `cpu`): PyTorch device
  - Options: `cpu`, `cuda`, `cuda:0`, etc.

- **`~dtype`** (default: `float32`): Floating point precision
  - Options: `float32`, `float64`
  - `float32` is recommended for speed

- **`~warmup_iters`** (default: `3`): Number of dummy forward passes at startup
  - Helps remove one-time first-call latency (especially with `use_compile`/GPU)
  - Set to `0` to disable

- **`~use_jit`** (default: `false`): Enable JIT tracing for value-only path
  - Can improve inference speed after warmup
  - Jacobians always use eager mode

- **`~use_compile`** (default: `false`): Enable `torch.compile()` for value-only path
  - Requires PyTorch 2.0+
  - Can significantly improve speed after compilation
  - First few calls will be slow during compilation

## Performance Tips

1. **Use float32**: Unless you need double precision, `float32` is 2x faster
   ```bash
   roslaunch torch_bridge torch_service_v3B.launch dtype:=float32
   ```

2. **Enable compilation** (PyTorch 2.0+): After initial compilation overhead, inference is much faster
   ```bash
   roslaunch torch_bridge torch_service_v3B.launch use_compile:=true
   ```

3. **Use GPU if available**: For complex models, GPU can be faster
   ```bash
   roslaunch torch_bridge torch_service_v3B.launch device:=cuda
   ```

4. **Profile first**: Run the smoke test to understand baseline performance
   ```bash
   python test_sysid_adapter.py --exp_dir /path/to/exp --num_calls 1000
   ```

## Choosing eager vs torch.compile vs JIT

This server supports three execution modes for the **value-only** inference path:

- **Eager (default)**: `use_compile:=false use_jit:=false`
- **torch.compile**: `use_compile:=true` (PyTorch 2.0+)
- **TorchScript tracing (JIT)**: `use_jit:=true`

Important: **Jacobians always run in eager mode** (autograd), regardless of these flags.

### Eager (no compile / no JIT)
- **Benefits**:
  - Most robust (fewest “it broke because of graph capture” surprises)
  - Fast startup (no compilation)
  - Best for debugging / rapid iteration when you’re changing models/configs often
- **Limitations**:
  - Typically not the fastest steady-state latency
  - First-call latency can still be higher than steady state (kernel selection / caching), so keep `warmup_iters > 0` for real-time use

### torch.compile (recommended when you can afford warmup)
- **Benefits**:
  - Often the best steady-state latency/throughput (especially on GPU)
  - Good fit for real-time loops once the compilation has happened
- **Limitations**:
  - **Warmup cost**: first few calls can be much slower due to compilation
  - Requires PyTorch **2.0+**
  - Can be sensitive to model/data-dependent control flow or changing shapes/dtypes (may recompile or fall back)
  - Harder to debug than eager when something goes wrong

### JIT tracing (TorchScript)
- **Benefits**:
  - Can improve steady-state latency vs eager with lower “magic” than compile
  - Often easier to deploy than compile in some environments
- **Limitations**:
  - Tracing captures one execution path; dynamic control flow may be incorrect
  - Not all PyTorch ops/models trace cleanly; you may get silent fallbacks or incorrect behavior
  - Can be slower than `torch.compile` on modern PyTorch

### Practical recommendation for your planner/controller
- Start with **eager + `warmup_iters:=3`**.
- If value-only inference is too slow, try **`use_compile:=true`** next.
- Use **`use_jit:=true`** mainly as a fallback when `torch.compile` is unavailable or unstable in your environment.

## Model Support

The integration supports all model configurations without hardcoding:

- **Model types**: `structured`, `direct`
- **Control adapters**: with or without, all feature flags
- **Friction learning**: with or without, all parameterization modes
- **Residual learning**: with or without
- **All other config flags**: via `populate_config_defaults()`

Simply point `exp_dir` to any valid experiment directory!

## Jacobian Notes

When `compute_jacobians=true`:
- Jacobians are computed w.r.t. **raw physical inputs** (not normalized)
- Chain rule is applied internally to account for normalization
- Output format matches `query_utils.hpp` expectations:
  - First block: `d(xd_next)/d(xd0)` as 9 values (row-major 3x3)
  - Second block: `d(xd_next)/d(ut)` as 6 values (row-major 3x2)
- Jacobians always use eager mode (not JIT/compiled)
- Typical overhead: 2-5x slower than value-only inference

## Troubleshooting

### Import Error: mushr_mujoco_sysid not found

Make sure `mushr_mujoco_sysid` is installed in the Python environment used by ROS:
```bash
# Check which Python is being used
which python

# Install in that environment
cd /path/to/mushr_mujoco_sysid
pip install -e .
```

Or add to `PYTHONPATH` in the launch file.

### Python environment mismatch (wrong torch / missing deps)

The launch files run the server via the wrapper script:
- `scripts/torch_service_server_wrapper.sh`

Edit `CUSTOM_PYTHON=...` in that script to point to the correct environment.

### Model loading fails

Verify all required files exist:
```bash
ls /path/to/exp_dir/
# Should show: config.json, best.pt, standardizers.json
```

### Slow inference

- Run the smoke test with timing: `--num_calls 1000`
- Try enabling `use_compile:=true`
- For very fast inference (<1ms), use GPU with compilation

### Jacobian mismatch

Run the numerical jacobian test:
```bash
python test_sysid_adapter.py --exp_dir /path/to/exp
```

If numerical and analytical jacobians differ significantly, this may indicate:
- Normalization chain rule issue
- Model architecture problem
- Numerical precision issues

## Files

- `scripts/sysid_model_adapter.py`: Model loading and inference adapter
- `scripts/torch_service_server.py`: ROS service server implementation
- `scripts/test_sysid_adapter.py`: Smoke test (no ROS required)
- `launch/torch_service_sysid.launch`: Generic launch file
- `launch/torch_service_v3B.launch`: Convenience launch for best model
- `README_SYSID.md`: This file

## Example: Using in C++

```cpp
#include <ros/ros.h>
#include <torch_bridge/query_utils.hpp>
#include <torch_bridge/TorchQuery.h>

ros::ServiceClient torch_client = nh.serviceClient<torch_bridge::TorchQuery>("/torch/service");
torch_bridge::TorchQuery srv;

srv.request.inputs = 2;
srv.request.input_dimensions = {3, 2};
srv.request.compute_jacobians = true;

// Pack inputs
Eigen::Vector3d xd0(1.0, 0.0, 0.0);  // vx, vy, w
Eigen::Vector2d ut(0.5, 0.2);         // vel_cmd, steer_cmd
torch_bridge::update_request(srv, xd0, ut);

// Call service
if (torch_client.call(srv)) {
    // Extract result
    Eigen::Vector3d xd_next;
    torch_bridge::get_result(srv, xd_next);
    
    // Extract jacobians if computed
    if (srv.request.compute_jacobians) {
        Eigen::Matrix3d jac_x;
        Eigen::Matrix<double, 3, 2> jac_u;
        torch_bridge::get_jacobian(srv, jac_x, jac_u);
    }
}
```
