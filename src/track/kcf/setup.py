from setuptools import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy as np

# Obtain the numpy include directory.  This logic works across numpy versions.
try:
    numpy_include = np.get_include()
except AttributeError:
    numpy_include = np.get_numpy_include()

extra_compile_args = ['-std=c++11']
ext_modules = [
    Extension(
        "fhog_util",
        ["fhog_util.pyx"],
        extra_compile_args = extra_compile_args,
        include_dirs = [numpy_include]
    ),
]

setup(
    name='fhog',
    ext_modules=ext_modules,
    cmdclass={'build_ext': build_ext},
)


