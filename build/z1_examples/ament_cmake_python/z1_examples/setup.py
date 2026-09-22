from setuptools import find_packages
from setuptools import setup

setup(
    name='z1_examples',
    version='0.0.0',
    packages=find_packages(
        include=('z1_examples', 'z1_examples.*')),
)
