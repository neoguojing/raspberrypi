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

        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)

        self.input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        self.output_shape = tuple(self.engine.get_tensor_shape(self.output_name))

        self.input_dtype = np.uint8
        self.output_dtype = np.int32

        self.input_size = int(np.prod(self.input_shape))
        self.output_size = int(np.prod(self.output_shape))

        # pinned memory
        self.host_input = cuda.pagelocked_empty(self.input_size, self.input_dtype)
        self.host_output = cuda.pagelocked_empty(self.output_size, self.output_dtype)

        self.output_view = self.host_output.reshape(self.output_shape)

        # device memory
        self.device_input = cuda.mem_alloc(self.host_input.nbytes)
        self.device_output = cuda.mem_alloc(self.host_output.nbytes)

        # bind
        self.context.set_tensor_address(self.input_name, int(self.device_input))
        self.context.set_tensor_address(self.output_name, int(self.device_output))

    def infer(self, frame_uint8: np.ndarray):

        # 1. 保证 contiguous
        frame_uint8 = np.ascontiguousarray(frame_uint8)

        # 2. dtype 检查
        assert frame_uint8.dtype == self.input_dtype

        # 3. shape 检查
        if frame_uint8.ndim == 3:
            assert frame_uint8.shape == self.input_shape[1:]
        else:
            assert frame_uint8.shape == self.input_shape

        # 4. H2D
        cuda.memcpy_htod_async(
            self.device_input,
            frame_uint8,
            self.stream
        )

        # 5. inference
        self.context.execute_async_v3(self.stream.handle)

        # 6. D2H
        cuda.memcpy_dtoh_async(
            self.host_output,
            self.device_output,
            self.stream
        )

        # 7. sync
        self.stream.synchronize()

        return self.host_output