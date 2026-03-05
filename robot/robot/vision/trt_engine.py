import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np


class TRTEngine:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.ERROR)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # 获取输入/输出名称和形状
        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)

        self.input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        self.output_shape = tuple(self.engine.get_tensor_shape(self.output_name))

        # ✅ 动态获取 dtype（关键修复！）
        self.input_dtype = trt.nptype(self.engine.get_tensor_dtype(self.input_name))
        self.output_dtype = trt.nptype(self.engine.get_tensor_dtype(self.output_name))

        # 分配内存
        self.input_size = int(np.prod(self.input_shape))
        self.output_size = int(np.prod(self.output_shape))

        self.host_input = cuda.pagelocked_empty(self.input_size, self.input_dtype)
        self.host_output = cuda.pagelocked_empty(self.output_size, self.output_dtype)
        self.output_view = self.host_output.reshape(self.output_shape)

        self.device_input = cuda.mem_alloc(self.host_input.nbytes)
        self.device_output = cuda.mem_alloc(self.host_output.nbytes)

        # 绑定地址
        self.context.set_tensor_address(self.input_name, int(self.device_input))
        self.context.set_tensor_address(self.output_name, int(self.device_output))

    def infer(self, frame_float32: np.ndarray):
        # ✅ 推荐：强制 4D 输入（最清晰）
        assert frame_float32.ndim == 4, f"Input must be 4D, got {frame_float32.ndim}D"
        assert frame_float32.shape == self.input_shape, f"Shape mismatch: {frame_float32.shape} != {self.input_shape}"
        assert frame_float32.dtype == self.input_dtype, f"Dtype mismatch"

        frame_float32 = np.ascontiguousarray(frame_float32)

        cuda.memcpy_htod_async(self.device_input, frame_float32, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.host_output, self.device_output, self.stream)
        self.stream.synchronize()

        return self.output_view  # 已 reshape，可直接索引