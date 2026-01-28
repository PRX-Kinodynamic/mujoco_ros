#!/common/home/st1122/Projects/mushr_mujoco_sysid/env/bin/python
"""
TorchServer: ROS service for real-time neural dynamics model inference.

Loads a trained mushr_mujoco_sysid model and serves predictions via TorchQuery service.
Supports value-only inference and optional jacobian computation.
"""

import os
import sys
import traceback

import numpy as np
import rospy
from torch_bridge.srv import TorchQuery, TorchQueryResponse

from sysid_model_adapter import SysidModelAdapter


class TorchServer:
    """ROS service server for sysid model inference."""

    def __init__(self):
        # Get ROS parameters
        self.service_name = rospy.get_param("~service_name", "/torch/service")
        self.exp_dir = rospy.get_param("~exp_dir")
        self.dt = rospy.get_param("~dt", 0.05)
        self.device = rospy.get_param("~device", "cpu")
        self.dtype = rospy.get_param("~dtype", "float32")
        self.use_jit = rospy.get_param("~use_jit", False)

        # Smart defaults: "auto" means None (adapter will choose based on device)
        use_compile_param = rospy.get_param("~use_compile", "auto")
        self.use_compile = (
            None if use_compile_param == "auto" else bool(use_compile_param)
        )

        self.use_tf32 = rospy.get_param("~use_tf32", False)

        use_cudagraph_param = rospy.get_param("~use_cudagraph", "auto")
        self.use_cudagraph = (
            None if use_cudagraph_param == "auto" else bool(use_cudagraph_param)
        )

        self.warmup_iters = rospy.get_param("~warmup_iters", 3)

        # Validate exp_dir
        if not os.path.exists(self.exp_dir):
            rospy.logfatal(f"Experiment directory does not exist: {self.exp_dir}")
            sys.exit(1)

        config_path = os.path.join(self.exp_dir, "config.json")
        ckpt_path = os.path.join(self.exp_dir, "best.pt")
        std_path = os.path.join(self.exp_dir, "standardizers.json")

        missing = []
        if not os.path.exists(config_path):
            missing.append("config.json")
        if not os.path.exists(ckpt_path):
            missing.append("best.pt")
        if not os.path.exists(std_path):
            missing.append("standardizers.json")

        if missing:
            rospy.logfatal(
                f"Missing required files in {self.exp_dir}: {', '.join(missing)}"
            )
            sys.exit(1)

        rospy.loginfo("=" * 70)
        rospy.loginfo("TorchServer Initializing...")
        rospy.loginfo(f"  Python executable: {sys.executable}")
        rospy.loginfo(f"  Python version: {sys.version.split()[0]}")
        rospy.loginfo(f"  Experiment dir: {self.exp_dir}")
        rospy.loginfo(f"  Service name: {self.service_name}")
        rospy.loginfo(f"  dt: {self.dt}")
        rospy.loginfo(f"  device: {self.device}")
        rospy.loginfo(f"  dtype: {self.dtype}")
        rospy.loginfo(f"  use_jit: {self.use_jit}")
        compile_str = (
            "auto (smart default)"
            if self.use_compile is None
            else str(self.use_compile)
        )
        rospy.loginfo(f"  use_compile: {compile_str}")
        rospy.loginfo(f"  use_tf32: {self.use_tf32}")
        cudagraph_str = (
            "auto (CUDA only)"
            if self.use_cudagraph is None
            else str(self.use_cudagraph)
        )
        rospy.loginfo(f"  use_cudagraph: {cudagraph_str}")
        rospy.loginfo(f"  warmup_iters: {self.warmup_iters}")

        # Initialize model adapter
        try:
            self.adapter = SysidModelAdapter(
                exp_dir=self.exp_dir,
                dt=self.dt,
                device=self.device,
                dtype=self.dtype,
                use_jit=self.use_jit,
                use_compile=self.use_compile,
                use_tf32=self.use_tf32,
                use_cudagraph=self.use_cudagraph,
                warmup_iters=self.warmup_iters,
            )
            info = self.adapter.get_info()
            rospy.loginfo("Model loaded successfully:")
            rospy.loginfo(f"  Model type: {info['model_type']}")
            rospy.loginfo(f"  Control adapter: {info['control_adapter_enabled']}")
            rospy.loginfo(f"  Learn friction: {info['learn_friction']}")
            rospy.loginfo(f"  Learn residual: {info['learn_residual']}")
        except Exception as e:
            rospy.logfatal(f"Failed to load model: {e}")
            traceback.print_exc()
            sys.exit(1)

        # Create ROS service
        self.service = rospy.Service(
            self.service_name, TorchQuery, self.service_callback
        )

        rospy.loginfo("TorchServer ready!")
        rospy.loginfo("=" * 70)

        # Statistics
        self._call_count = 0
        self._jacobian_call_count = 0

        self.sum_durations   = [0.0]*2
        self.total_durations = [0]*2
        self.max_duration = [0.0]*2
        self.min_duration = [0.0]*2

    def service_callback(self, req):
        """Handle TorchQuery service requests."""
        start = rospy.Time.now()
        response = TorchQueryResponse()

        try:
            # Validate request format
            if req.inputs != 2:
                rospy.logwarn_throttle(
                    10.0,
                    f"Expected inputs=2, got inputs={req.inputs}. "
                    "Adapter expects (xd0, ut).",
                )

            if len(req.input_dimensions) != 2:
                rospy.logerr(
                    f"Expected 2 input dimensions, got {len(req.input_dimensions)}"
                )
                return response

            dim_xd0 = req.input_dimensions[0]
            dim_ut = req.input_dimensions[1]

            if dim_xd0 != 3 or dim_ut != 2:
                rospy.logwarn_throttle(
                    10.0,
                    f"Expected dims [3, 2], got [{dim_xd0}, {dim_ut}]. "
                    "Proceeding but may fail.",
                )

            # Extract data
            if len(req.data) < 5:
                rospy.logerr(
                    f"Insufficient data: expected at least 5 elements, got {len(req.data)}"
                )
                return response

            xd0 = np.array(req.data[:3], dtype=np.float64)
            ut = np.array(req.data[3:5], dtype=np.float64)

            jacs = 0
            # Inference
            if req.compute_jacobians:
                jacs = 1
                # Compute both value and jacobians
                xd_next = self.adapter.predict(xd0, ut)
                jac_x, jac_u = self.adapter.jacobians(xd0, ut)

                # Pack response
                response.outputs = 1
                response.output_dimensions = [3]
                response.result = xd_next.tolist()

                # Pack jacobians: first (3x3) row-major, then (3x2) row-major
                jacobians = []
                for i in range(3):
                    for j in range(3):
                        jacobians.append(float(jac_x[i, j]))
                for i in range(3):
                    for j in range(2):
                        jacobians.append(float(jac_u[i, j]))

                response.jacobians = jacobians

                self._jacobian_call_count += 1
            else:
                # Value-only inference
                xd_next = self.adapter.predict(xd0, ut)

                response.outputs = 1
                response.output_dimensions = [3]
                response.result = xd_next.tolist()

            self._call_count += 1

            # Log statistics periodically
            if self._call_count % 1000 == 0:
                avg_0 = self.sum_durations[0] / self.total_durations[0] if (self.total_durations[0] > 0) else 0.0
                avg_1 = self.sum_durations[1] / self.total_durations[1] if (self.total_durations[1] > 0) else 0.0
                rospy.loginfo(
                    f"Processed {self._call_count} calls "
                    f"({self._jacobian_call_count} with jacobians)."
                    f"\tJacobians Without \t With "
                    f"\tmin: {self.min_duration[0]}, {self.min_duration[1]}\n"
                    f"\tmax: {self.max_duration[0]}, {self.max_duration[1]}\n"
                    f"\tmean: {avg_0}, {avg_1} \n"
                )

        except Exception as e:
            rospy.logerr(f"Service callback error: {e}")
            traceback.print_exc()
            # Return empty response on error
            response.outputs = 0
            response.output_dimensions = []
            response.result = []
            response.jacobians = []

        end = rospy.Time.now()
        curr_duration = (end - start).to_sec()
        self.sum_durations[jacs] += curr_duration
        self.total_durations[jacs] += 1
        self.max_duration[jacs] = max(self.max_duration[jacs], curr_duration)
        self.min_duration[jacs] = min(self.max_duration[jacs], curr_duration)
        
        return response


if __name__ == "__main__":
    rospy.init_node("TorchServer")
    try:
        ros_node = TorchServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
