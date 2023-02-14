from setuptools import setup
from Cython.Build import cythonize

from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules = cythonize("CheckCollision2.pyx", compiler_directives={'language_level' : "3"})   # or "2" or "3str")

)