from setuptools import find_packages
from setuptools import setup

package_name = 'ros2bag_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Marcel Zeilinger',
    author_email='marcel.zeilinger@ait.ac.at',
    maintainer='Marcel Zeilinger',
    maintainer_email='marcel.zeilinger@ait.ac.at',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Tool extensions for ros2bag cli',
    long_description="""\
The package provides the additional commands for the ROS 2 bag command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2bag.verb': [
            'convert = ros2bag_tools.verb.convert:ConvertVerb',
            'cut = ros2bag_tools.verb.cut:CutVerb',
            'extract = ros2bag_tools.verb.extract:ExtractVerb',
            'reframe = ros2bag_tools.verb.reframe:ReframeVerb',
            'replace = ros2bag_tools.verb.replace:ReplaceVerb',
            'restamp = ros2bag_tools.verb.restamp:RestampVerb',
            'process = ros2bag_tools.verb.process:ProcessVerb',
        ],
    }
)
