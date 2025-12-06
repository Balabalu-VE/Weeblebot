from setuptools import find_packages, setup

package_name = 'sensor_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weeblebot',
    maintainer_email='nicolasdittmarg1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_IMU = sensor_data.get_IMU:main',
            'get_enc = sensor_data.get_enc:main',
        ],
    },
)
