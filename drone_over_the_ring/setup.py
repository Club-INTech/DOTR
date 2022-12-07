from setuptools import setup, find_packages

setup(
    name='drone_over_the_ring',
    version='0.0.1',
    install_requires=[
        'importlib-metadata; python_version == "3.8"',
    ],
    packages=find_packages(
        where='.',
        include=['drone_over_the_ring*'],
        exclude=['drone_over_the_ring.tests'],
    ),
)
