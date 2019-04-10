"""
cython compilation file for the models and cacu cons
this needs re-running when switching between 32 bit and 64 bit devices
(delphin2 is 32 bit)
"""
from distutils.core import setup
from Cython.Build import cythonize
import numpy as np

setup(
    name = 'cacu_cons',
    ext_modules = cythonize("cacu_cons.pyx")
    )

setup(
    name = 'auv_model_high',
    ext_modules = cythonize("auv_model_high.pyx")
    )

setup(
    name = 'auv_model_low',
    ext_modules = cythonize("auv_model_low.pyx")
    )

setup(
    name = 'auv_model_mul',
    ext_modules = cythonize("auv_model_mul.pyx"),
    )
