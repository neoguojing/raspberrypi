import numpy as np
import tensorrt as trt
import pycuda.driver as cuda

class TRTEngine:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        # 动态引擎不再初始化固定 host/device 内存
        # 我们将在推理时根据需要进行分配或复用
        self.bindings = {}
        self.allocations = {} # 缓存 device 内存地址

    def infer(self, input_tensor):
        """
        input_tensor: np.ndarray, 比如 (1, 3, 720, 1280)
        """
        # 1. 动态设置当前输入的 Shape
        input_name = self.engine.get_tensor_name(0) # 假设第一个是输入
        self.context.set_input_shape(input_name, input_tensor.shape)
        
        # 2. 准备所有 Tensor 的地址（输入 + 输出）
        # 3090 显存大，这里可以做简单的动态适配
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.context.get_tensor_shape(name)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = int(np.prod(shape))
            nbytes = size * np.dtype(dtype).itemsize
            
            # 如果尺寸变了，重新分配显存 (Device Memory)
            if name not in self.allocations or self.allocations[name].size < nbytes:
                self.allocations[name] = cuda.mem_alloc(nbytes)
            
            self.context.set_tensor_address(name, int(self.allocations[name]))

        # 3. 数据拷贝：Host -> Device
        input_device = self.allocations[input_name]
        # 确保数据连续
        input_data = np.ascontiguousarray(input_tensor).astype(trt.nptype(self.engine.get_tensor_dtype(input_name)))
        cuda.memcpy_htod_async(input_device, input_data, self.stream)

        # 4. 执行推理
        self.context.execute_async_v3(self.stream.handle)

        # 5. 数据拷贝：Device -> Host
        output_name = self.engine.get_tensor_name(1) # 假设第二个是输出
        output_shape = self.context.get_tensor_shape(output_name)
        output_host = np.empty(output_shape, dtype=trt.nptype(self.engine.get_tensor_dtype(output_name)))
        
        cuda.memcpy_dtoh_async(output_host, self.allocations[output_name], self.stream)
        self.stream.synchronize()
        
        return output_host