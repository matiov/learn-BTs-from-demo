from setuptools import setup, find_packages

package_name = 'py_trees'

setup(
    name=package_name,
    version='0.0.0',
    maintainer='Matteo Iovino',
    maintainer_email='matteo.iovino@se.abb.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'operator',
        'uuid',
        'pydot',
        'inspect',
        'functools',
        'itertools',
        'signal',
        'numbers',
        'threading',
        'multiprocessing',
        'traceback',
    ],
    tests_require=['pytest'],
)
