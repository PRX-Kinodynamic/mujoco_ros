#!/usr/bin/env python
"""
SysidModelAdapter: Load and run mushr_mujoco_sysid models for real-time inference.

This adapter:
- Loads experiment artifacts (config.json, best.pt, standardizers.json)
- Builds the correct model via mushr_mujoco_sysid.model_factory
- Provides fast value inference predict(xd0, ut) -> xd_next
- Optionally computes jacobians w.r.t. raw inputs
- Optionally uses FastInferenceSession for GPU-optimized inference
"""

import json
import os
from typing import Dict, Tuple
import numpy as np
import torch
import warnings

# Import mushr_mujoco_sysid modules
try:
    from mushr_mujoco_sysid.config_utils import populate_config_defaults
    from mushr_mujoco_sysid.model_factory import build_model
    from mushr_mujoco_sysid.utils import load_standardizers_json
    from mushr_mujoco_sysid.models.system_models import (
        StructuredDynamicsModel,
        DirectDynamicsModel,
    )

    # Try to import fast inference session (optional, for GPU-optimized path)
    try:
        from mushr_mujoco_sysid.fast import FastInferenceSession

        _HAS_FAST_SESSION = True
    except ImportError:
        _HAS_FAST_SESSION = False
except ImportError as e:
    raise ImportError(
        f"Failed to import mushr_mujoco_sysid: {e}\n"
        "Make sure mushr_mujoco_sysid is installed or in PYTHONPATH."
    )


class SysidModelAdapter:
    """
    Adapter for loading and running sysid models in real-time.

    Supports all model configurations (structured, direct, with/without adapters, etc.)
    without hardcoding for any specific model variant.
    """

    def __init__(
        self,
        exp_dir: str,
        dt: float,
        device: str = "cpu",
        dtype: str = "float32",
        use_jit: bool = False,
        use_compile: bool = None,  # Auto: True by default
        use_tf32: bool = False,
        use_cudagraph: bool = None,  # Auto: True on CUDA, False on CPU
        warmup_iters: int = 3,
    ):
        """
        Initialize the adapter by loading model artifacts.

        Args:
            exp_dir: Path to experiment directory containing config.json, best.pt, standardizers.json
            dt: Fixed timestep for forward dynamics
            device: 'cpu' or 'cuda'
            dtype: 'float32' or 'float64'
            use_jit: Whether to JIT trace the model (value-only path, fallback only)
            use_compile: Whether to use torch.compile (default: True for best performance)
            use_tf32: Enable TF32 for matmul/convs (Ampere+ GPUs)
            use_cudagraph: Enable CUDA Graph replay (default: True on CUDA, False on CPU)
            warmup_iters: Number of warmup iterations
        """
        self.exp_dir = exp_dir
        self.dt = dt
        self.device = torch.device(device)
        self.dtype = torch.float32 if dtype == "float32" else torch.float64
        self.use_jit = use_jit
        self.warmup_iters = int(warmup_iters)

        # Smart defaults: compile everywhere, cudagraph on CUDA only
        if use_compile is None:
            use_compile = True  # Default to compile for best performance
        if use_cudagraph is None:
            use_cudagraph = self.device.type == "cuda"  # Auto-enable on CUDA

        self.use_compile = use_compile
        self.use_tf32 = use_tf32
        self.use_cudagraph = use_cudagraph

        # Validate CUDA Graph requirements
        if self.use_cudagraph and self.device.type != "cuda":
            warnings.warn(
                "use_cudagraph requested but device is not CUDA; disabling CUDA Graph"
            )
            self.use_cudagraph = False

        # Decide whether to use FastInferenceSession (single compile boundary)
        # Use it if available and if compile/tf32/cudagraph is requested
        self._fast_session = None
        use_fast_path = _HAS_FAST_SESSION and (
            use_compile or use_tf32 or self.use_cudagraph
        )

        if use_fast_path:
            print("[SysidModelAdapter] Using FastInferenceSession for value inference")
            try:
                self._fast_session = FastInferenceSession(
                    exp_dir=exp_dir,
                    dt=dt,
                    device=device,
                    dtype=dtype,
                    use_compile=use_compile,
                    use_tf32=use_tf32,
                    use_cudagraph=self.use_cudagraph,
                    warmup_iters=warmup_iters,
                )
                # Store references from fast session for jacobian fallback
                self.model = self._fast_session.model
                self.input_std = self._fast_session.input_std
                self.target_std = self._fast_session.target_std
                self.config = self._fast_session.config
                self._input_mean = self._fast_session._input_mean
                self._input_std_val = self._fast_session._input_std_val
                self._target_mean = self._fast_session._target_mean
                self._target_std_val = self._fast_session._target_std_val
                self._dt_tensor = self._fast_session._dt_tensor
                # Fast session handles value inference; we're done initializing
                return
            except Exception as e:
                print(
                    f"[SysidModelAdapter] FastInferenceSession failed: {e}, falling back to standard path"
                )
                self._fast_session = None

        # Standard fallback path (for jacobians, CPU-only, or if fast session unavailable)
        print("[SysidModelAdapter] Using standard inference path")

        # Load config
        config_path = os.path.join(exp_dir, "config.json")
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config not found: {config_path}")

        with open(config_path, "r") as f:
            self.config = json.load(f)

        # Populate defaults for backward compatibility
        self.config = populate_config_defaults(self.config)

        # Load standardizers
        std_path = os.path.join(exp_dir, "standardizers.json")
        if not os.path.exists(std_path):
            raise FileNotFoundError(f"Standardizers not found: {std_path}")

        self.input_std, self.target_std = load_standardizers_json(std_path)

        # Build model
        self.model = build_model(self.config, self.device)
        self.model.eval()

        # Load checkpoint
        ckpt_name = self.config.get("training", {}).get("ckpt_name", "best.pt")
        ckpt_path = os.path.join(exp_dir, ckpt_name)
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

        checkpoint = torch.load(ckpt_path, map_location=self.device)

        # Handle both raw state_dict and checkpoint dict
        if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
            state_dict = checkpoint["model_state_dict"]
        elif isinstance(checkpoint, dict) and "model_state" in checkpoint:
            state_dict = checkpoint["model_state"]
        else:
            state_dict = checkpoint

        self.model.load_state_dict(state_dict)
        self.model.to(self.device, dtype=self.dtype)

        # Pre-allocate buffers for batch=1 inference
        self._xd0_buf = torch.zeros((1, 3), device=self.device, dtype=self.dtype)
        self._ut_buf = torch.zeros((1, 2), device=self.device, dtype=self.dtype)
        self._dt_tensor = torch.tensor([self.dt], device=self.device, dtype=self.dtype)

        # Convert standardizer stats to device tensors
        self._input_mean = torch.tensor(
            self.input_std.mean, device=self.device, dtype=self.dtype
        )
        self._input_std_val = torch.tensor(
            self.input_std.std, device=self.device, dtype=self.dtype
        )
        self._target_mean = torch.tensor(
            self.target_std.mean, device=self.device, dtype=self.dtype
        )
        self._target_std_val = torch.tensor(
            self.target_std.std, device=self.device, dtype=self.dtype
        )

        # Optional: JIT or compile for value-only path
        self._compiled_model = None
        if use_compile and hasattr(torch, "compile"):
            try:
                self._compiled_model = torch.compile(self.model)
            except Exception as e:
                print(f"Warning: torch.compile failed: {e}, falling back to eager")
        elif use_jit:
            try:
                # Trace with dummy inputs
                dummy_xd0 = torch.zeros((1, 3), device=self.device, dtype=self.dtype)
                dummy_ut = torch.zeros((1, 2), device=self.device, dtype=self.dtype)
                with torch.inference_mode():
                    self._compiled_model = torch.jit.trace(
                        self.model, (dummy_xd0, dummy_ut, self._dt_tensor[0])
                    )
            except Exception as e:
                print(f"Warning: JIT trace failed: {e}, falling back to eager")

        if self.warmup_iters > 0:
            self._warmup(self.warmup_iters)

    def _warmup(self, iters: int) -> None:
        model_to_use = (
            self._compiled_model if self._compiled_model is not None else self.model
        )
        xd0_norm = torch.zeros((1, 3), device=self.device, dtype=self.dtype)
        ut_norm = torch.zeros((1, 2), device=self.device, dtype=self.dtype)

        with torch.inference_mode():
            for _ in range(int(iters)):
                if isinstance(self.model, StructuredDynamicsModel):
                    _ = model_to_use(xd0_norm, ut_norm, self._dt_tensor[0])
                elif isinstance(self.model, DirectDynamicsModel):
                    _ = model_to_use(xd0_norm, ut_norm, self._dt_tensor)
                else:
                    _ = model_to_use(xd0_norm, ut_norm, self._dt_tensor[0])

    def predict(self, xd0: np.ndarray, ut: np.ndarray) -> np.ndarray:
        """
        Fast value-only inference: predict next velocity state.

        Args:
            xd0: Current velocity state [vx, vy, w], shape (3,)
            ut: Control input [vel_cmd, steer_cmd], shape (2,)

        Returns:
            xd_next: Predicted next velocity state, shape (3,)
        """
        # If fast session is available, use it (single compile boundary)

        if self._fast_session is not None:
            return self._fast_session.predict_one_numpy(xd0, ut)

        # Standard fallback path
        # Copy to pre-allocated buffers
        self._xd0_buf[0] = torch.from_numpy(xd0).to(self.device, dtype=self.dtype)
        self._ut_buf[0] = torch.from_numpy(ut).to(self.device, dtype=self.dtype)

        with torch.inference_mode():
            # Normalize inputs (first 5 dims: xd0 + ut)
            xu_raw = torch.cat([self._xd0_buf, self._ut_buf], dim=1)
            xu_norm = (xu_raw - self._input_mean) / self._input_std_val

            xd0_norm = xu_norm[:, :3]
            ut_norm = xu_norm[:, 3:5]

            # Forward pass
            model_to_use = (
                self._compiled_model if self._compiled_model is not None else self.model
            )

            if isinstance(self.model, StructuredDynamicsModel):
                xd_next_norm = model_to_use(xd0_norm, ut_norm, self._dt_tensor[0])
            elif isinstance(self.model, DirectDynamicsModel):
                # DirectDynamicsModel expects dt as part of input
                xd_next_norm = model_to_use(xd0_norm, ut_norm, self._dt_tensor)
            else:
                xd_next_norm = model_to_use(xd0_norm, ut_norm, self._dt_tensor[0])

            # Denormalize output
            xd_next = xd_next_norm * self._target_std_val + self._target_mean


        return xd_next[0].cpu().numpy()

    def jacobians(
        self, xd0: np.ndarray, ut: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute jacobians of xd_next w.r.t. raw inputs.

        Note: Always uses eager/autograd path (not compiled), regardless of
        whether FastInferenceSession is available.

        Args:
            xd0: Current velocity state [vx, vy, w], shape (3,)
            ut: Control input [vel_cmd, steer_cmd], shape (2,)

        Returns:
            jac_x: d(xd_next)/d(xd0), shape (3, 3)
            jac_u: d(xd_next)/d(ut), shape (3, 2)
        """
        # If using fast session for value inference, fall back to eager for jacobians
        if self._fast_session is not None:
            warnings.warn(
                "Jacobians computed using eager/autograd path (not compiled). "
                "For best performance, consider value-only inference for real-time use.",
                UserWarning,
            )

        # Create tensors with gradients enabled
        xd0_t = torch.from_numpy(xd0).to(self.device, dtype=self.dtype).unsqueeze(0)
        ut_t = torch.from_numpy(ut).to(self.device, dtype=self.dtype).unsqueeze(0)
        xd0_t.requires_grad_(True)
        ut_t.requires_grad_(True)

        # Normalize inputs
        xu_raw = torch.cat([xd0_t, ut_t], dim=1)
        xu_norm = (xu_raw - self._input_mean) / self._input_std_val

        xd0_norm = xu_norm[:, :3]
        ut_norm = xu_norm[:, 3:5]

        # Forward pass (always use eager model for jacobians)
        if isinstance(self.model, StructuredDynamicsModel):
            xd_next_norm = self.model(xd0_norm, ut_norm, self._dt_tensor[0])
        elif isinstance(self.model, DirectDynamicsModel):
            xd_next_norm = self.model(xd0_norm, ut_norm, self._dt_tensor)
        else:
            xd_next_norm = self.model(xd0_norm, ut_norm, self._dt_tensor[0])

        # Denormalize output
        xd_next = xd_next_norm * self._target_std_val + self._target_mean

        # Compute jacobians using autograd
        # We need d(xd_next)/d(xd0_raw) and d(xd_next)/d(ut_raw)
        jac_x_list = []
        jac_u_list = []

        for i in range(3):  # For each output dimension
            # Clear gradients
            if xd0_t.grad is not None:
                xd0_t.grad.zero_()
            if ut_t.grad is not None:
                ut_t.grad.zero_()

            # Compute gradient of output[i] w.r.t. inputs
            xd_next[0, i].backward(retain_graph=(i < 2))

            # Extract gradients
            jac_x_list.append(xd0_t.grad[0].cpu().numpy().copy())
            jac_u_list.append(ut_t.grad[0].cpu().numpy().copy())

        jac_x = np.stack(jac_x_list, axis=0)  # (3, 3)
        jac_u = np.stack(jac_u_list, axis=0)  # (3, 2)

        return jac_x, jac_u

    def get_info(self) -> Dict[str, any]:
        """Return information about the loaded model."""
        # If using fast session, delegate to it
        if self._fast_session is not None:
            info = self._fast_session.get_info()
            info["using_fast_session"] = True
            return info

        # Standard path
        return {
            "exp_dir": self.exp_dir,
            "model_type": self.config.get("model", {}).get("type", "unknown"),
            "dt": self.dt,
            "device": str(self.device),
            "dtype": str(self.dtype),
            "use_jit": self.use_jit,
            "use_compile": self.use_compile,
            "use_tf32": self.use_tf32,
            "use_cudagraph": self.use_cudagraph,
            "using_fast_session": False,
            "control_adapter_enabled": self.config.get("model", {})
            .get("control_adapter", {})
            .get("enabled", False),
            "learn_friction": self.config.get("model", {}).get("learn_friction", False),
            "learn_residual": self.config.get("model", {}).get("learn_residual", False),
        }
