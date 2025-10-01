from setuptools import setup
from Cython.Build import cythonize
import glob
import os

# 모든 하위 디렉토리 포함한 py 파일 리스트 생성
pkgename="angle_detection"
py_files = glob.glob(f"/home/keti/Desktop/PoC_Talos/APP_Talos/APP_Base/APP/detector/angle_detection.py", recursive=True)

setup(
    name=pkgename,
    ext_modules=cythonize(
        py_files,
        compiler_directives={"language_level": "3"},
    ),
    zip_safe=False,
)