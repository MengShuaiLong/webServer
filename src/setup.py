from setuptools import setup

package_name = 'webServer'

setup(
    name=package_name,
    version='0.21.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='msl',
    author_email='msl@osrfoundation.org',
    maintainer='msl',
    maintainer_email='msl@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='webServer.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'subscriber_old_school ='
            # ' examples_rclpy_minimal_subscriber.subscriber_old_school:main',
            'server = webServer.server:main',
            # 'subscriber_member_function ='
            # ' examples_rclpy_minimal_subscriber.subscriber_member_function:main',
        ],
    },
)
