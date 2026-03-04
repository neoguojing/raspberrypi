import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # 自动初始化 CUDA 上下文

class TRTEngine:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # 静态引擎：在初始化时确定所有 binding 的 shape 和 dtype
        self.input_name = None
        self.output_name = None
        self.host_inputs = []
        self.host_outputs = []
        self.device_inputs = []
        self.device_outputs = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)  # 静态 shape 直接从 engine 获取
            size = int(np.prod(shape))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_name = name
                self.host_inputs.append(host_mem)
                self.device_inputs.append(device_mem)
            else:
                self.output_name = name
                self.host_outputs.append(host_mem)
                self.device_outputs.append(device_mem)

        assert self.input_name is not None, "No input tensor found!"
        assert self.output_name is not None, "No output tensor found!"

    def infer(self, input_tensor: np.ndarray):
        """
        input_tensor: np.ndarray with fixed shape (e.g., (1, 3, 512, 512))
        Returns: output np.ndarray
        """
        # 1. 检查输入 shape 是否匹配（静态引擎必须严格匹配）
        expected_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        if input_tensor.shape != expected_shape:
            raise ValueError(
                f"Input shape {input_tensor.shape} does not match engine expected shape {expected_shape}"
            )

        # 2. Host -> Device
        np.copyto(self.host_inputs[0], input_tensor.ravel())
        cuda.memcpy_htod_async(self.device_inputs[0], self.host_inputs[0], self.stream)

        # 3. 设置 tensor 地址（静态引擎只需设一次，但每次 inference 前设也无妨）
        self.context.set_tensor_address(self.input_name, int(self.device_inputs[0]))
        self.context.set_tensor_address(self.output_name, int(self.device_outputs[0]))

        # 4. 执行推理
        self.context.execute_async_v3(self.stream.handle)

        # 5. Device -> Host
        cuda.memcpy_dtoh_async(self.host_outputs[0], self.device_outputs[0], self.stream)
        self.stream.synchronize()

        # 6. 重塑输出 shape
        output_shape = tuple(self.engine.get_tensor_shape(self.output_name))
        return self.host_outputs[0].reshape(output_shape)