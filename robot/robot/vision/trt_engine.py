import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt


class TRTEngine:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.ERROR)
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(logger)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)

        self.input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        self.output_shape = tuple(self.engine.get_tensor_shape(self.output_name))

        if -1 in self.input_shape or -1 in self.output_shape:
            raise RuntimeError(
                "This runtime is configured for static-shape engines only, "
                f"but got input={self.input_shape}, output={self.output_shape}"
            )

        self.input_dtype = trt.nptype(self.engine.get_tensor_dtype(self.input_name))
        self.output_dtype = trt.nptype(self.engine.get_tensor_dtype(self.output_name))

        input_size = int(np.prod(self.input_shape))
        output_size = int(np.prod(self.output_shape))

        self.host_output = cuda.pagelocked_empty(output_size, self.output_dtype)
        self.output_view = self.host_output.reshape(self.output_shape)

        self.device_input = cuda.mem_alloc(input_size * np.dtype(self.input_dtype).itemsize)
        self.device_output = cuda.mem_alloc(output_size * np.dtype(self.output_dtype).itemsize)

        self.context.set_tensor_address(self.input_name, int(self.device_input))
        self.context.set_tensor_address(self.output_name, int(self.device_output))

    def infer(self, frame_float32: np.ndarray):
        assert frame_float32.ndim == 4, f"Input must be 4D, got {frame_float32.ndim}D"

        if frame_float32.dtype != self.input_dtype:
            frame_float32 = frame_float32.astype(self.input_dtype, copy=False)

        frame_float32 = np.ascontiguousarray(frame_float32)
        assert tuple(frame_float32.shape) == self.input_shape, (
            f"Shape mismatch: got {tuple(frame_float32.shape)}, expected {self.input_shape}"
        )

        cuda.memcpy_htod_async(self.device_input, frame_float32, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.host_output, self.device_output, self.stream)
        self.stream.synchronize()

        return self.output_view
