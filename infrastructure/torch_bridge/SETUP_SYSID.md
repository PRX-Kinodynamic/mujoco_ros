# Setup Guide: Sysid Model Integration

This guide walks you through setting up the Python environment and testing the sysid model server.

## Prerequisites

- ROS (Melodic/Noetic)
- Python 3.6+
- PyTorch
- mushr_mujoco_sysid package

## Step 1: Python Environment Setup

The server uses a dedicated conda environment to isolate dependencies from the rest of ROS.

### Option A: Use existing mushr_mujoco_sysid environment

If you already have a conda environment for `mushr_mujoco_sysid`:

```bash
# Activate the environment
conda activate /common/home/st1122/Projects/mushr_mujoco_sysid/env

# Verify mushr_mujoco_sysid is importable
python -c "from mushr_mujoco_sysid import build_model; print('OK')"

# Verify ROS Python bindings are available
python -c "import rospy; print('OK')"
```

If `rospy` is not available, you may need to install it:
```bash
conda install -c conda-forge ros-rospy
# or
pip install rospkg catkin_pkg
```

### Option B: Create a new environment

```bash
# Create environment
conda create -n torch_ros python=3.9

# Activate
conda activate torch_ros

# Install PyTorch (adjust for your CUDA version)
conda install pytorch torchvision torchaudio cpuonly -c pytorch
# OR for GPU:
# conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia

# Install other dependencies
conda install numpy

# Install mushr_mujoco_sysid
cd /common/home/st1122/Projects/mushr_mujoco_sysid
pip install -e .

# Install ROS Python bindings
pip install rospkg catkin_pkg
```

## Step 2: Configure the wrapper (recommended)

The ROS launch files run the server via the wrapper script:

- `/common/home/st1122/Projects/ros_workspace/src/infrastructure/torch_bridge/scripts/torch_service_server_wrapper.sh`

This guarantees the correct Python environment is used regardless of ROS’s default Python.

Update this line to point at the Python you want ROS to use:

```bash
CUSTOM_PYTHON="/common/home/st1122/Projects/mushr_mujoco_sysid/env/bin/python"
```

## Step 3: Test Without ROS (Smoke Test)

Before running the full ROS service, test the adapter standalone:

```bash
cd /common/home/st1122/Projects/ros_workspace/src/infrastructure/torch_bridge/scripts

# Basic smoke test
python test_sysid_adapter.py \
  --exp_dir /common/home/st1122/Projects/mushr_mujoco_sysid/experiments/experiments-v3/v3B_struct_h10_tf0_seed4/v3B_struct_h10_tf0_seed4 \
  --dt 0.05 \
  --device cpu

# Expected output:
# ======================================================================
# SysidModelAdapter Smoke Test
# ======================================================================
# ...
# TEST: Basic Inference
# ...
# Timing statistics over 100 calls:
#   Min:    X.XXX ms
#   Max:    X.XXX ms
#   Mean:   X.XXX ms
# ...
# ALL TESTS PASSED
# ======================================================================
```

If this fails, check:
1. The experiment directory path is correct
2. Required files exist (config.json, best.pt, standardizers.json)
3. mushr_mujoco_sysid is installed and importable

## Step 4: Build ROS Package

```bash
cd /common/home/st1122/Projects/ros_workspace

# Build the package (if needed)
catkin_make -DCATKIN_WHITELIST_PACKAGES="torch_bridge"
# or
catkin build torch_bridge

# Source the workspace
source devel/setup.bash
```

## Step 5: Launch the Service

### Test with rosrun (for debugging)

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Run the server directly
cd /common/home/st1122/Projects/ros_workspace
source devel/setup.bash

rosrun torch_bridge torch_service_server_wrapper.sh \
  _service_name:=/torch/service \
  _exp_dir:=/common/home/st1122/Projects/mushr_mujoco_sysid/experiments/experiments-v3/v3B_struct_h10_tf0_seed4/v3B_struct_h10_tf0_seed4 \
  _dt:=0.05 \
  _device:=cpu
```

Expected output:
```
======================================================================
TorchServer Initializing...
  Python executable: /path/to/python
  Python version: 3.x.x
  Experiment dir: /path/to/exp
  Service name: /torch/service
  dt: 0.05
  device: cpu
  dtype: float32
  use_jit: False
  use_compile: False
Model loaded successfully:
  Model type: structured
  Control adapter: True
  Learn friction: False
  Learn residual: True
TorchServer ready!
======================================================================
```

### Test with roslaunch

```bash
# Using the convenience launch file for v3B model
roslaunch torch_bridge torch_service_v3B.launch

# Or with custom experiment
roslaunch torch_bridge torch_service_sysid.launch \
  exp_dir:=/path/to/your/experiment \
  dt:=0.05 \
  device:=cpu
```

## Step 6: Test the Service

In another terminal:

```bash
# Check service is available
rosservice list | grep torch
# Should show: /torch/service

# Test with C++ timing node (if compiled)
rosrun torch_bridge torch_service_timing _total_calls:=100

# Expected output:
# Without Jacobians
#   Min: X.XXX
#   Max: X.XXX
#   Mean: X.XXX
# With Jacobians
#   Min: X.XXX
#   Max: X.XXX
#   Mean: X.XXX
```

## Step 7: Test Different Configurations

### GPU inference
```bash
roslaunch torch_bridge torch_service_v3B.launch device:=cuda
```

### With torch.compile (PyTorch 2.0+)
```bash
roslaunch torch_bridge torch_service_v3B.launch use_compile:=true
```

### With JIT tracing
```bash
roslaunch torch_bridge torch_service_v3B.launch use_jit:=true
```

### Notes on compile/JIT (practical)

- `use_compile:=true` usually gives the best steady-state latency, but startup will be slower (compilation). Keep `warmup_iters` enabled so the first “real” query doesn’t pay the cost.
- `use_jit:=true` can help in some environments, but may be less robust than eager and often slower than `torch.compile` on PyTorch 2.x.
- Jacobians (when `compute_jacobians=true`) always run in eager mode.

### With different experiment
```bash
roslaunch torch_bridge torch_service_sysid.launch \
  exp_dir:=/path/to/v3C_struct_h20_tf0_seed4/v3C_struct_h20_tf0_seed4 \
  dt:=0.05
```

## Troubleshooting

### Problem: "Failed to import mushr_mujoco_sysid"

**Solution**: Make sure the package is installed in the Python environment:
```bash
# Check which Python the shebang points to
head -1 /path/to/torch_service_server.py

# Use that Python
/path/to/that/python -c "from mushr_mujoco_sysid import build_model"
```

If it fails:
```bash
cd /common/home/st1122/Projects/mushr_mujoco_sysid
/path/to/that/python -m pip install -e .
```

### Problem: "rospy not found"

**Solution**: Install ROS Python bindings in your conda environment:
```bash
conda activate your_env
pip install rospkg catkin_pkg
```

### Problem: Service starts but no output

**Check**:
1. Is roscore running?
2. Are there any Python errors in the terminal?
3. Try running the smoke test first to isolate the issue

### Problem: Inference is slow (>10ms per call)

**Solutions**:
1. Use `float32` instead of `float64`: `dtype:=float32`
2. Enable compilation: `use_compile:=true` (first call will be slow)
3. Use GPU if available: `device:=cuda`
4. Profile with smoke test: `python test_sysid_adapter.py --num_calls 1000`

### Problem: Jacobians don't match numerical approximation

**Check**:
1. Run the full smoke test to see the difference
2. If difference is small (<1e-3), it may be numerical precision
3. If difference is large, report as potential bug

## Environment Variables (Alternative to Shebang)

If you don't want to hardcode the Python path in the shebang, you can use a wrapper script:

Edit `scripts/torch_service_server_wrapper.sh`:
```bash
#!/bin/bash
source /path/to/your/conda.sh
conda activate your_env
python /path/to/torch_service_server.py "$@"
```

Then in the launch file, call the wrapper instead of the Python script directly.

## Next Steps

Once the service is running:
1. Integrate with your planner/controller
2. Monitor performance in real-time scenarios
3. Test with different models/configurations
4. Profile jacobian computation if needed for your application

For more details, see `README_SYSID.md`.
