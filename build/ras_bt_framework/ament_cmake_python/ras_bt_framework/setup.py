from setuptools import find_packages
from setuptools import setup

setup(
    name='ras_bt_framework',
    version='0.0.0',
    packages=find_packages(
        include=('ras_bt_framework', 'ras_bt_framework.*')),
)
