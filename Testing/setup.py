from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

ext_modules = [
    Pybind11Extension(
        "kalman_filter",
        sources=["bindings.cpp", "../src/KalmanFilter.cpp"],
        include_dirs=["../include"],
    ),
]

setup(
    name="kalman_filter",
    version="0.1.0",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
