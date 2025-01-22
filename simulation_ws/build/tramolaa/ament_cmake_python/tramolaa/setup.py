from setuptools import find_packages
from setuptools import setup

setup(
    name='tramolaa',
    version='0.0.0',
    packages=find_packages(
        include=('tramolaa', 'tramolaa.*')),
)
