from setuptools import setup

package_name = 'sensor_msgs_py'

setup(
    name=package_name,
    version='5.3.6',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tully Foote',
    maintainer_email='tfoote@openrobotics.org',
    author='Sebastian Grans',
    author_email='sebastian.grans@gmail.com',
    description='A package for easy creation and reading of PointCloud2 messages in Python.',
    license='BSD',
    tests_require=['pytest'],
)
