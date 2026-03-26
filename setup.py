from setuptools import setup, find_packages
from pathlib import Path

setup(
    name='MAVCOT',
    version='0.1dev',
    author='David Ingraham',
    author_email='davingrahamd@gmail.com',
    license='GNU GPL V3',
    long_description=Path('README.md').read_text(encoding='utf-8'),
    packages=find_packages(),
    scripts=['mavcot/mavcot_proxy.py'],
    include_package_data=True,
    install_requires=['pymavlink'],
)