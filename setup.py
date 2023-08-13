from setuptools import setup

package_name = 'ik_benchmarking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mohamed Raessa',
    author_email='mohamed.s.raessa@gmail.com',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'run_ik_benchmarking = ik_benchmarking.ik_benchmarking_data_generator:main',
        ],
    }
)
