#!/usr/bin/env python
import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _maybe_add_mushr_repo_to_syspath(exp_dir: str) -> str | None:
    p = Path(exp_dir).resolve()
    for parent in (p, *p.parents):
        if (parent / "mushr_mujoco_sysid" / "__init__.py").exists():
            sys.path.insert(0, str(parent))
            return str(parent)
    return None


def _maybe_add_this_dir_to_syspath() -> None:
    this_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(this_dir))


@dataclass(frozen=True)
class Variant:
    name: str
    use_jit: bool
    use_compile: bool
    use_cudagraph: bool


def _percentile(x: np.ndarray, q: float) -> float:
    return float(np.percentile(x, q))


def _effective_key(info: dict) -> tuple:
    """
    Collapse variants that are effectively identical in this adapter.

    - If using FastInferenceSession, JIT is not used (standard-path only).
    """
    using_fast = bool(info.get("using_fast_session", False))
    eff_use_compile = bool(info.get("use_compile", False))
    eff_use_cudagraph = bool(info.get("use_cudagraph", False))
    eff_use_jit = bool(info.get("use_jit", False)) if not using_fast else False
    return (str(info.get("device", "")), using_fast, eff_use_jit, eff_use_compile, eff_use_cudagraph)


def _make_requested_variants() -> list[Variant]:
    # All combinations of {jit, compile, cudagraph}, with readable names.
    variants: list[Variant] = []
    for use_jit in (False, True):
        for use_compile in (False, True):
            for use_cudagraph in (False, True):
                if not (use_jit or use_compile or use_cudagraph):
                    name = "none"
                else:
                    parts = []
                    if use_jit:
                        parts.append("jit")
                    if use_compile:
                        parts.append("compile")
                    if use_cudagraph:
                        parts.append("cudagraph")
                    name = "+".join(parts)
                variants.append(
                    Variant(
                        name=name,
                        use_jit=use_jit,
                        use_compile=use_compile,
                        use_cudagraph=use_cudagraph,
                    )
                )
    return variants


def _print_summary(rows: list[dict[str, object]]) -> None:
    if not rows:
        print("\nNo benchmark results collected.")
        return

    def sort_key(r: dict[str, object]) -> float:
        return float(r["mean_ms"])

    print("\n" + "=" * 78)
    print("Summary (sorted by mean latency)")
    print("=" * 78)

    devices = sorted({str(r["device"]) for r in rows})
    for dev in devices:
        dev_rows = [r for r in rows if str(r["device"]) == dev]
        dev_rows.sort(key=sort_key)
        best = dev_rows[0]
        print(f"\nDevice: {dev}  (best: {best['variant']} @ {best['mean_ms']:.3f} ms)")
        for r in dev_rows:
            print(
                f"  - {str(r['variant']):22s}  "
                f"mean={float(r['mean_ms']):7.3f}  "
                f"p50={float(r['p50_ms']):7.3f}  "
                f"p90={float(r['p90_ms']):7.3f}  "
                f"p99={float(r['p99_ms']):7.3f}  "
                f"fast={bool(r['using_fast_session'])}"
            )

    overall = sorted(rows, key=sort_key)
    best_overall = overall[0]
    print(
        f"\nOverall best: {best_overall['device']} | {best_overall['variant']} "
        f"@ {best_overall['mean_ms']:.3f} ms"
    )


def _run_bench(adapter, *, num_warmup: int, num_iters: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    xd0 = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    ut = np.array([0.5, 0.2], dtype=np.float64)

    for _ in range(int(num_warmup)):
        xd0_varied = xd0 + rng.normal(scale=0.1, size=(3,))
        ut_varied = ut + rng.normal(scale=0.05, size=(2,))
        _ = adapter.predict(xd0_varied, ut_varied)

    is_cuda = getattr(adapter, "device", None) is not None and adapter.device.type == "cuda"
    if is_cuda:
        import torch

        torch.cuda.synchronize()

    times_ms = np.zeros((int(num_iters),), dtype=np.float64)
    for i in range(int(num_iters)):
        xd0_varied = xd0 + rng.normal(scale=0.1, size=(3,))
        ut_varied = ut + rng.normal(scale=0.05, size=(2,))
        t0 = time.perf_counter()
        _ = adapter.predict(xd0_varied, ut_varied)
        if is_cuda:
            import torch

            torch.cuda.synchronize()
        t1 = time.perf_counter()
        times_ms[i] = (t1 - t0) * 1000.0

    return times_ms


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark SysidModelAdapter latency across backends."
    )
    parser.add_argument("--exp_dir", type=str, required=True)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--dtype", type=str, default="float32", choices=["float32", "float64"])
    parser.add_argument(
        "--devices",
        type=str,
        default="auto",
        choices=["auto", "cpu", "cuda", "both"],
        help="Which devices to benchmark (default: auto)",
    )
    parser.add_argument("--num_warmup", type=int, default=50)
    parser.add_argument("--num_iters", type=int, default=300)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--csv_out", type=str, default="")
    args = parser.parse_args()

    _maybe_add_this_dir_to_syspath()
    added = _maybe_add_mushr_repo_to_syspath(args.exp_dir)
    if added is not None:
        print(f"[bench] Added mushr repo to PYTHONPATH: {added}")

    from sysid_model_adapter import SysidModelAdapter

    variants = _make_requested_variants()

    want_cuda = args.devices in ("cuda", "both", "auto")
    want_cpu = args.devices in ("cpu", "both", "auto")

    cuda_available = False
    if want_cuda:
        try:
            import torch

            cuda_available = torch.cuda.is_available()
        except Exception:
            cuda_available = False

    devices: list[str] = []
    if want_cpu:
        devices.append("cpu")
    if want_cuda and cuda_available:
        devices.append("cuda")
    if want_cuda and not cuda_available and args.devices in ("cuda", "both"):
        print("[bench] CUDA requested but not available.")
        return 2

    rows: list[dict[str, object]] = []

    print("=" * 78)
    print("SysidModelAdapter latency benchmark")
    print("=" * 78)
    print(f"exp_dir: {args.exp_dir}")
    print(f"dt: {args.dt}")
    print(f"dtype: {args.dtype}")
    print(f"devices: {devices}")
    print(f"warmup iters: {args.num_warmup}")
    print(f"bench iters: {args.num_iters}")
    print("")

    for device in devices:
        seen_effective: set[tuple] = set()
        for v in variants:
            if device == "cpu" and v.use_cudagraph:
                print(f"[{device:4s} | {v.name:9s}] SKIP (CUDA-only)")
                continue

            print(f"[{device:4s} | {v.name:9s}] loading...")
            adapter = SysidModelAdapter(
                exp_dir=args.exp_dir,
                dt=args.dt,
                device=device,
                dtype=args.dtype,
                use_jit=v.use_jit,
                use_compile=v.use_compile,
                use_cudagraph=v.use_cudagraph,
            )

            info = adapter.get_info()
            used_fast = bool(info.get("using_fast_session", False))
            eff = _effective_key(info)
            if eff in seen_effective:
                print(f"[{device:4s} | {v.name:9s}] SKIP (duplicate effective config)")
                continue
            seen_effective.add(eff)
            if v.use_cudagraph and device == "cuda" and not used_fast:
                print(f"[{device:4s} | {v.name:9s}] SKIP (no FastInferenceSession)")
                continue

            times_ms = _run_bench(
                adapter,
                num_warmup=args.num_warmup,
                num_iters=args.num_iters,
                seed=args.seed,
            )

            row = {
                "device": device,
                "variant": v.name,
                "using_fast_session": used_fast,
                "mean_ms": float(times_ms.mean()),
                "p50_ms": _percentile(times_ms, 50),
                "p90_ms": _percentile(times_ms, 90),
                "p99_ms": _percentile(times_ms, 99),
                "min_ms": float(times_ms.min()),
                "max_ms": float(times_ms.max()),
                "effective": eff,
            }
            rows.append(row)

            print(
                f"[{device:4s} | {v.name:9s}] "
                f"mean={row['mean_ms']:.3f} ms  "
                f"p50={row['p50_ms']:.3f}  p90={row['p90_ms']:.3f}  p99={row['p99_ms']:.3f}  "
                f"(fast={used_fast})"
            )

    _print_summary(rows)

    if args.csv_out:
        out_path = Path(args.csv_out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "device",
            "variant",
            "using_fast_session",
            "mean_ms",
            "p50_ms",
            "p90_ms",
            "p99_ms",
            "min_ms",
            "max_ms",
            "effective",
        ]
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in rows:
                writer.writerow(r)
        print(f"\n[bench] Wrote CSV: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

