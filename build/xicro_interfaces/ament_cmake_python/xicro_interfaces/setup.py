from setuptools import find_packages
from setuptools import setup

setup(
    name='xicro_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('xicro_interfaces', 'xicro_interfaces.*')),
)
