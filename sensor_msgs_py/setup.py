from setuptools import setup

package_name = 'sensor_msgs_py'

setup(
    name=package_name,
    version='2.2.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carroll',
    maintainer_email='michael@openrobotics.org',
    author='Sebastian Grans',
    author_email='sebastian.grans@gmail.com',
    description='A package for easy creation and reading of PointCloud2 messages in Python.',
    license='BSD',
    tests_require=['pytest'],
)
