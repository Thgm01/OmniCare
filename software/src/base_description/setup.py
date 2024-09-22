from setuptools import setup
import os
from glob import glob

package_name = 'base_description'

data_files=[
   ('share/ament_index/resource_index/packages',
       ['resource/' + package_name]),
   ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):
   paths_dict = {}
   for directory in directory_list:
       for (path, directories, filenames) in os.walk(directory):
           for filename in filenames:
               file_path = os.path.join(path, filename)
               install_path = os.path.join('share', package_name, path)
               if install_path in paths_dict.keys():
                   paths_dict[install_path].append(file_path)
               else:
                   paths_dict[install_path] = [file_path]
   for key in paths_dict.keys():
       data_files.append((key, paths_dict[key]))
   return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, [ 'config/', 'launch/','meshes/', 'simulation/', 'urdf/']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    #     (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    #     (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    #     (os.path.join('share', package_name, 'config'), glob('config/*')),
    #     (os.path.join('share', package_name, 'simulation'), glob('simulation/models/box/*')),
    #     (os.path.join('share', package_name, 'simulation'),glob('simulation/models/simple_room/*')),
    #     (os.path.join('share', package_name, 'simulation'),glob('simulation/models/wood_square/*')),
    #     (os.path.join('share', package_name, 'simulation'), glob('simulation/worlds/*'))
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
