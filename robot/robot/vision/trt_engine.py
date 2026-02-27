import numpy as np

class TRTEngine:
    def __init__(self, engine_path):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401
        import pycuda.driver as cuda

        self.cuda = cuda
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"TensorRT engine load failed: {engine_path}")

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.input_shape = None
        self.output_shape = None
        self.input_host = None
        self.output_host = None
        self.input_device = None
        self.output_device = None

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = tuple(self.context.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            if -1 in shape:
                raise ValueError(f"TensorRT engine must be static shape, got {name}: {shape}")
            size = int(np.prod(shape))
            host = cuda.pagelocked_empty(size, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.context.set_tensor_address(name, int(device))
            if mode == trt.TensorIOMode.INPUT:
                self.input_shape = shape
                self.input_host = host
                self.input_device = device
            else:
                self.output_shape = shape
                self.output_host = host
                self.output_device = device

        if self.input_shape is None or self.output_shape is None:
            raise RuntimeError("TensorRT engine must contain exactly one input and one output tensor")

    def infer(self, input_tensor):
        np.copyto(self.input_host, input_tensor.ravel())
        self.cuda.memcpy_htod_async(self.input_device, self.input_host, self.stream)
        ok = self.context.execute_async_v3(stream_handle=self.stream.handle)
        if not ok:
            raise RuntimeError("TensorRT execute_async_v3 failed")
        self.cuda.memcpy_dtoh_async(self.output_host, self.output_device, self.stream)
        self.stream.synchronize()
        return self.output_host.reshape(self.output_shape)

