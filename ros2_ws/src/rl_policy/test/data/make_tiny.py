"""tiny.onnx 만들기 (테스트용, 한 번): y = x·W + b · 입력 obs [1, 3] · 출력 actions [1, 2]. Isaac venv(onnx)에서."""
import os

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

w = numpy_helper.from_array(np.array([[1, 0], [0, 1], [1, -1]], dtype=np.float32), "W")
b = numpy_helper.from_array(np.array([0.5, -0.5], dtype=np.float32), "B")
g = helper.make_graph([helper.make_node("Gemm", ["obs", "W", "B"], ["actions"])], "tiny",
                      [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 3])],
                      [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 2])], [w, b])
m = helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)])
m.ir_version = 9
onnx.save(m, os.path.join(os.path.dirname(os.path.abspath(__file__)), "tiny.onnx"))
