from setuptools import find_packages, setup

package_name = 'TEL'

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
    maintainer='diegochencoconut',
    maintainer_email='diegochencoconut@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_sub = TEL.wheel_sub:main',
            'wheel_test_pub = TEL.wheel_test_pub:main',
            'stepper_sub = TEL.stepper_sub:main',
            'stepper_test_pub = TEL.stepper_test_pub:main',
            'shooter_CIM_sub = TEL.shooter_CIM_sub:main',
            'shooter_CIM_test_pub = TEL.shooter_CIM_test_pub:main',
            'shooter_servo_sub = TEL.shooter_servo_sub:main',
            'shooter_servo_test_pub = TEL.shooter_servo_test_pub:main',
            'load1_servo_sub = TEL.load1_servo_sub:main',
            'load1_servo_test_pub = TEL.load1_servo_test_pub:main',
            'up_linear_sub = TEL.up_linear_sub:main',
            'bottom_linear_sub = TEL.bottom_linear_sub:main',
            'bottom_linear_test_pub = TEL.bottom_linear_test_pub:main',
            'up_linear_test_pub = TEL.up_linear_test_pub:main',
            'camera_servo_test_pub = TEL.camera_servo_test_pub:main',
            'camera_servo_sub = TEL.camera_servo_sub:main',
            'server = TEL.server.server:main',
        ],
    },
)
