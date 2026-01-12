#!/usr/bin/env python
"""
Smoke test for SysidModelAdapter.

Tests model loading, inference, and jacobian computation without ROS.
Useful for rapid iteration and debugging.

Usage:
    python test_sysid_adapter.py --exp_dir /path/to/experiment
"""

import argparse
import time
import numpy as np

from sysid_model_adapter import SysidModelAdapter


def test_basic_inference(adapter: SysidModelAdapter, num_calls: int = 100):
    """Test basic value-only inference."""
    print("\n" + "=" * 70)
    print("TEST: Basic Inference")
    print("=" * 70)

    # Random initial states
    xd0 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    ut = np.array([0.5, 0.2], dtype=np.float64)

    print(f"Input xd0: {xd0}")
    print(f"Input ut: {ut}")

    # First call (may include JIT compilation if enabled)
    start = time.time()
    xd_next = adapter.predict(xd0, ut)
    first_time = time.time() - start

    print(f"Output xd_next: {xd_next}")
    print(f"First call time: {first_time * 1000:.3f} ms")

    # Multiple calls for timing
    times = []
    for _ in range(num_calls):
        # Vary inputs slightly
        xd0_varied = xd0 + np.random.randn(3) * 0.1
        ut_varied = ut + np.random.randn(2) * 0.05

        start = time.time()
        xd_next = adapter.predict(xd0_varied, ut_varied)
        elapsed = time.time() - start
        times.append(elapsed)

    times = np.array(times) * 1000  # Convert to ms

    print(f"\nTiming statistics over {num_calls} calls:")
    print(f"  Min:    {times.min():.3f} ms")
    print(f"  Max:    {times.max():.3f} ms")
    print(f"  Mean:   {times.mean():.3f} ms")
    print(f"  Median: {np.median(times):.3f} ms")
    print(f"  Std:    {times.std():.3f} ms")

    if times.mean() > 10.0:
        print("\nWARNING: Mean inference time > 10ms. Consider enabling JIT/compile.")


def test_jacobians(adapter: SysidModelAdapter):
    """Test jacobian computation."""
    print("\n" + "=" * 70)
    print("TEST: Jacobian Computation")
    print("=" * 70)

    xd0 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    ut = np.array([0.5, 0.2], dtype=np.float64)

    print(f"Input xd0: {xd0}")
    print(f"Input ut: {ut}")

    start = time.time()
    jac_x, jac_u = adapter.jacobians(xd0, ut)
    elapsed = time.time() - start

    print("\nJacobian d(xd_next)/d(xd0) (3x3):")
    print(jac_x)
    print("\nJacobian d(xd_next)/d(ut) (3x2):")
    print(jac_u)
    print(f"\nJacobian computation time: {elapsed * 1000:.3f} ms")

    # Sanity checks
    assert jac_x.shape == (3, 3), f"Expected jac_x shape (3,3), got {jac_x.shape}"
    assert jac_u.shape == (3, 2), f"Expected jac_u shape (3,2), got {jac_u.shape}"

    # Check for non-trivial jacobians (not all zeros)
    if np.allclose(jac_x, 0.0) and np.allclose(jac_u, 0.0):
        print("\nWARNING: Jacobians are all zero! This may indicate a problem.")
    else:
        print("\nJacobians look reasonable (non-zero).")


def test_numerical_jacobian(adapter: SysidModelAdapter, eps: float = 1e-5):
    """Verify jacobians using finite differences."""
    print("\n" + "=" * 70)
    print("TEST: Numerical Jacobian Verification")
    print("=" * 70)

    xd0 = np.array([1.0, 0.1, 0.05], dtype=np.float64)
    ut = np.array([0.5, 0.2], dtype=np.float64)

    # Compute analytical jacobians
    jac_x_analytical, jac_u_analytical = adapter.jacobians(xd0, ut)

    # Compute numerical jacobian w.r.t. xd0
    jac_x_numerical = np.zeros((3, 3))
    xd_baseline = adapter.predict(xd0, ut)

    for i in range(3):
        xd0_perturbed = xd0.copy()
        xd0_perturbed[i] += eps
        xd_perturbed = adapter.predict(xd0_perturbed, ut)
        jac_x_numerical[:, i] = (xd_perturbed - xd_baseline) / eps

    # Compute numerical jacobian w.r.t. ut
    jac_u_numerical = np.zeros((3, 2))

    for i in range(2):
        ut_perturbed = ut.copy()
        ut_perturbed[i] += eps
        xd_perturbed = adapter.predict(xd0, ut_perturbed)
        jac_u_numerical[:, i] = (xd_perturbed - xd_baseline) / eps

    # Compare
    print("Analytical Jx:")
    print(jac_x_analytical)
    print("\nNumerical Jx:")
    print(jac_x_numerical)
    print("\nDifference Jx (abs):")
    diff_x = np.abs(jac_x_analytical - jac_x_numerical)
    print(diff_x)
    print(f"Max difference: {diff_x.max():.6e}")

    print("\nAnalytical Ju:")
    print(jac_u_analytical)
    print("\nNumerical Ju:")
    print(jac_u_numerical)
    print("\nDifference Ju (abs):")
    diff_u = np.abs(jac_u_analytical - jac_u_numerical)
    print(diff_u)
    print(f"Max difference: {diff_u.max():.6e}")

    # Tolerance check
    tol = 1e-3
    if diff_x.max() < tol and diff_u.max() < tol:
        print(f"\nPASS: Jacobians match numerical approximation (tol={tol})")
    else:
        print(f"\nWARNING: Jacobians differ from numerical approximation by > {tol}")


def test_rollout(adapter: SysidModelAdapter, steps: int = 20):
    """Test a short rollout to verify stability."""
    print("\n" + "=" * 70)
    print("TEST: Short Rollout")
    print("=" * 70)

    xd = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    ut = np.array([0.5, 0.2], dtype=np.float64)

    print(f"Initial xd: {xd}")
    print(f"Control: {ut}")
    print(f"Rolling out {steps} steps...\n")

    trajectory = [xd.copy()]

    for step in range(steps):
        xd = adapter.predict(xd, ut)
        trajectory.append(xd.copy())

        if step < 5 or step >= steps - 2:
            print(
                f"  Step {step+1:2d}: xd = [{xd[0]:7.4f}, {xd[1]:7.4f}, {xd[2]:7.4f}]"
            )
        elif step == 5:
            print("  ...")

    trajectory = np.array(trajectory)

    # Check for NaN or Inf
    if np.any(np.isnan(trajectory)) or np.any(np.isinf(trajectory)):
        print("\nERROR: Trajectory contains NaN or Inf!")
    else:
        print("\nRollout stable (no NaN/Inf).")

    # Check for explosion
    if np.abs(trajectory).max() > 100.0:
        print("WARNING: Trajectory values are very large, possible instability.")


def test_cuda_graph(adapter: SysidModelAdapter, num_calls: int = 200):
    """Test optional CUDA Graph replay path (skips on CPU)."""
    print("\n" + "=" * 70)
    print("TEST: CUDA Graph Replay (optional)")
    print("=" * 70)

    # This test is only meaningful on CUDA.
    if getattr(adapter, "device", None) is None or adapter.device.type != "cuda":
        print("SKIP: device is not CUDA")
        return

    fast_session = getattr(adapter, "_fast_session", None)
    if fast_session is None:
        raise RuntimeError(
            "CUDA Graph requested but FastInferenceSession is not active "
            "(did the adapter fall back to the standard path?)"
        )

    if not getattr(fast_session, "use_cudagraph", False):
        raise RuntimeError(
            "CUDA Graph requested but FastInferenceSession.use_cudagraph is False"
        )
    if getattr(fast_session, "_cuda_graph", None) is None:
        raise RuntimeError("CUDA Graph requested but no CUDA graph was captured")

    # Basic functional check: different inputs should yield different outputs.
    xd0_a = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    ut_a = np.array([0.5, 0.2], dtype=np.float64)
    xd0_b = np.array([1.2, -0.1, 0.05], dtype=np.float64)
    ut_b = np.array([0.7, -0.1], dtype=np.float64)

    ya = adapter.predict(xd0_a, ut_a)
    yb = adapter.predict(xd0_b, ut_b)

    print(f"Output A: {ya}")
    print(f"Output B: {yb}")
    if np.allclose(ya, yb):
        raise RuntimeError(
            "CUDA Graph replay output did not change with input; this is unexpected"
        )

    # Basic stability check: run a bunch of calls without errors.
    for _ in range(int(num_calls)):
        _ = adapter.predict(xd0_a, ut_a)
    print(f"PASS: replay ran for {num_calls} calls without error")


def main():
    parser = argparse.ArgumentParser(description="Smoke test for SysidModelAdapter")
    parser.add_argument(
        "--exp_dir",
        type=str,
        required=True,
        help="Path to experiment directory (contains config.json, best.pt, standardizers.json)",
    )
    parser.add_argument(
        "--dt", type=float, default=0.05, help="Timestep (default: 0.05)"
    )
    parser.add_argument(
        "--device", type=str, default="cpu", help="Device: cpu or cuda (default: cpu)"
    )
    parser.add_argument(
        "--dtype",
        type=str,
        default="float32",
        help="float32 or float64 (default: float32)",
    )
    parser.add_argument("--use_jit", action="store_true", help="Enable JIT tracing")
    parser.add_argument(
        "--use_compile",
        type=lambda x: (
            None if x.lower() == "auto" else x.lower() in ("true", "1", "yes")
        ),
        default=None,
        help="Enable torch.compile (default: auto = smart default)",
    )
    parser.add_argument(
        "--use_cudagraph",
        type=lambda x: (
            None if x.lower() == "auto" else x.lower() in ("true", "1", "yes")
        ),
        default=None,
        help="Enable CUDA Graph replay (default: auto = CUDA only)",
    )
    parser.add_argument(
        "--num_calls", type=int, default=100, help="Number of timing calls"
    )
    parser.add_argument(
        "--skip_numerical_jacobian",
        action="store_true",
        help="Skip numerical jacobian verification (faster)",
    )

    args = parser.parse_args()

    print("=" * 70)
    print("SysidModelAdapter Smoke Test")
    print("=" * 70)
    print(f"Experiment dir: {args.exp_dir}")
    print(f"dt: {args.dt}")
    print(f"device: {args.device}")
    print(f"dtype: {args.dtype}")
    print(f"use_jit: {args.use_jit}")
    compile_str = (
        "auto (smart default)" if args.use_compile is None else str(args.use_compile)
    )
    print(f"use_compile: {compile_str}")
    cudagraph_str = (
        "auto (CUDA only)" if args.use_cudagraph is None else str(args.use_cudagraph)
    )
    print(f"use_cudagraph: {cudagraph_str}")

    # Initialize adapter
    print("\nLoading model...")
    adapter = SysidModelAdapter(
        exp_dir=args.exp_dir,
        dt=args.dt,
        device=args.device,
        dtype=args.dtype,
        use_jit=args.use_jit,
        use_compile=args.use_compile,
        use_cudagraph=args.use_cudagraph,
    )

    info = adapter.get_info()
    print("\nModel info:")
    for key, value in info.items():
        print(f"  {key}: {value}")

    # Run tests
    try:
        test_basic_inference(adapter, num_calls=args.num_calls)
        test_jacobians(adapter)
        if not args.skip_numerical_jacobian:
            test_numerical_jacobian(adapter)
        test_rollout(adapter, steps=20)
        if args.use_cudagraph:
            test_cuda_graph(adapter, num_calls=max(args.num_calls, 200))

        print("\n" + "=" * 70)
        print("ALL TESTS PASSED")
        print("=" * 70)

    except Exception as e:
        print("\n" + "=" * 70)
        print("TEST FAILED")
        print("=" * 70)
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
