from setuptools import find_packages
from setuptools import setup

setup(
    name='xicro_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('xicro_pkg', 'xicro_pkg.*')),
)
