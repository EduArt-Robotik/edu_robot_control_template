from setuptools import find_packages, setup

package_name = 'edu_nav2_example_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_controller_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.kiesel@eudart-robotik.com',
    description='Uses Simple Commander API to task robot to drive to three target poses via Nav2',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_py_controller = edu_nav2_example_py.nav2_py_controller:main'
        ],
    },
)
