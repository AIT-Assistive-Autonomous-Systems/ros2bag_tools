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
            'add = ros2bag_tools.verb.add:AddVerb',
            'cut = ros2bag_tools.verb.cut:CutVerb',
            'drop = ros2bag_tools.verb.drop:DropVerb',
            'echo = ros2bag_tools.verb.echo:EchoVerb',
            'export = ros2bag_tools.verb.export:ExportVerb',
            'extract = ros2bag_tools.verb.extract:ExtractVerb',
            'plot = ros2bag_tools.verb.plot:PlotVerb',
            'prune = ros2bag_tools.verb.prune:PruneVerb',
            'reframe = ros2bag_tools.verb.reframe:ReframeVerb',
            'rename = ros2bag_tools.verb.rename:RenameVerb',
            'replace = ros2bag_tools.verb.replace:ReplaceVerb',
            'restamp = ros2bag_tools.verb.restamp:RestampVerb',
            'sync = ros2bag_tools.verb.sync:SyncVerb',
            'summary = ros2bag_tools.verb.summary:SummaryVerb',
            'process = ros2bag_tools.verb.process:ProcessVerb',
            'video = ros2bag_tools.verb.video:VideoVerb',
        ],
        'ros2bag_tools.filter': [
            'add = ros2bag_tools.filter.add:AddFilter',
            'cut = ros2bag_tools.filter.cut:CutFilter',
            'drop = ros2bag_tools.filter.drop:DropFilter',
            'extract = ros2bag_tools.filter.extract:ExtractFilter',
            'image = ros2bag_tools.filter.image:ImageFilter',
            'prune = ros2bag_tools.filter.prune:PruneFilter',
            'reframe = ros2bag_tools.filter.reframe:ReframeFilter',
            'rename = ros2bag_tools.filter.rename:RenameFilter',
            'replace = ros2bag_tools.filter.replace:ReplaceFilter',
            'restamp = ros2bag_tools.filter.restamp:RestampFilter',
            'sync = ros2bag_tools.filter.sync:SyncFilter',
        ],
        'ros2bag_tools.exporter': [
            'tum_trajectory = ros2bag_tools.exporter.tum_trajectory:TUMTrajectoryExporter',
            'image = ros2bag_tools.exporter.image:ImageExporter',
            'stamp = ros2bag_tools.exporter.stamp:StampExporter',
            'pcd = ros2bag_tools.exporter.pcd:PcdExporter',
        ],
    }
)
