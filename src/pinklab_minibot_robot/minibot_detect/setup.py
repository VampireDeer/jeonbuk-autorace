from setuptools import find_packages, setup
import glob, os


package_name = 'minibot_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seok',
    maintainer_email='github.com/VampireDeer',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect = minibot_detect.aruco_detect:main',
            'object_detect = minibot_detect.object_detect:main',
            'all_detect = minibot_detect.all_detect:main'
        ],
    },
)
