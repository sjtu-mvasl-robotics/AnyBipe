from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

extensions = [
    Extension("jacobian", 
              sources=["cpp/jacobian_build.pyx", "cpp/jacobian.cpp"],
              include_dirs=["cpp"],
              language='c++',
              extra_compile_args=["-std=c++11"])
]

setup(
    name="JacobianCython",
    ext_modules=cythonize(extensions),
)
