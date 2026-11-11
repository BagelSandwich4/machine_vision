from setuptools import find_packages, setup

package_name = 'logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', '*.test']),
    package_data={package_name: ['calibration_data.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ophelia-lonzo, thanadetch-mateedunsatits',
    maintainer_email='ophelialonzo@gmail.com, thanadetch.m@gmail.com',
    description='Use Neatos and AprilTags to log items',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = logger.my_node:main',
            'tag_logger = logger.tag_logger:main',
            'april_raspicamtest = logger.april_raspicamtest:main',
        ],
    },
)
