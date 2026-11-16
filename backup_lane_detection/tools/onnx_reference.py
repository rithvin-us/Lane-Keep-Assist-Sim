"""Reference outputs of an ONNX model with onnxruntime (CPU) for the MATLAB parity tests.

Usage: python tools/onnx_reference.py <model.onnx> <input.bin> <out_prefix>
Input: raw little-endian float32 in C (NCHW) order, shape = model input.
Output: <out_prefix>_<k>.bin (k = 1..number of outputs, same encoding) and
<out_prefix>.json with the list of output shapes.
"""
import json
import sys

import numpy as np
import onnxruntime as ort


def main(onnx_path, in_bin, out_prefix):
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    shape = [d if isinstance(d, int) else 1 for d in inp.shape]
    x = np.fromfile(in_bin, dtype="<f4").reshape(shape)
    shapes = []
    for k, y in enumerate(sess.run(None, {inp.name: x}), start=1):
        y.astype("<f4").tofile(f"{out_prefix}_{k}.bin")
        shapes.append(list(y.shape))
    with open(out_prefix + ".json", "w") as f:
        json.dump(shapes, f)
    print("ok", shapes)


if __name__ == "__main__":
    main(*sys.argv[1:4])
