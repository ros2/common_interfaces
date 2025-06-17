from setuptools import setup

package_name = 'sensor_msgs_py'

setup(
    name=package_name,
    version='4.9.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geoffrey Biggs, Tully Foote',
    maintainer_email='geoff@openrobotics.org, tfoote@openrobotics.org',
    author='Sebastian Grans',
    author_email='sebastian.grans@gmail.com',
    description='A package for easy creation and reading of PointCloud2 messages in Python.',
    license='BSD',
    tests_require=['pytest'],
)
