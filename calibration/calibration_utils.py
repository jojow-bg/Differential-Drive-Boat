import json
import numpy as np


def load_calibration_json(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    invA = np.array(data["invA"], dtype=float)
    xbar = np.array(data["xbar"], dtype=float)

    return invA, xbar