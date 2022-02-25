from setuptools import setup, find_packages

package_name = 'bt_learning'

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
        'matplotlib',
        'numpy',
        'pickle',
        'behaviors',
        'shutil',
        'distutils',
        'webbrowser',
        'py_trees',
        'pydot',
        'threading',
        'pick',
        'simulation',
        'statistics',
        'scipy',
        'sklearn',
        'logging',
        'itertools',
    ],
    tests_require=['pytest'],
)
