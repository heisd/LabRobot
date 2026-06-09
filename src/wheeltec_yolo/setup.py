from setuptools import setup

package_name = 'wheeltec_yolo'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['launch/yolo.launch.py']))
data_files.append(('share/' + package_name, ['launch/yolo_follow.launch.py']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='Wheeltec 集成 yolo_ros: 便捷启动 + YOLO 3D 跟随(保持设定距离).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_follow = wheeltec_yolo.yolo_follow:main',
        ],
    },
)
