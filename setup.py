from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'capote'

def package_dir_tree(target_dir, base_install_path):
    """
    Recursively collects all files in target_dir and maps them
    to their corresponding install paths under base_install_path.
    """
    entries = {}
    for filepath in glob(os.path.join(target_dir, '**'), recursive=True):
        if os.path.isfile(filepath):
            relpath = os.path.relpath(filepath, start=target_dir)
            install_path = os.path.join(base_install_path, os.path.dirname(relpath))
            entries.setdefault(install_path, []).append(filepath)
    return list(entries.items())

data_files = [
    # Requerido pelo ROS2
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),

    # Adicionado para atender as demandas do nosso pacote!
    (f'share/{package_name}/launch', glob('launch/*.py')),
    (f'share/{package_name}/description', glob('description/*.urdf.xacro')),
    (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
]

# Adiciona todos os arquivos de modelos da pasta models/ recursivamente (se existir)
if os.path.isdir('models'):
    data_files.extend(package_dir_tree('models', f'share/{package_name}/models'))

# Adiciona todos os arquivos de modelos de mundo da pasta world/ recursivamente (se existir)
if os.path.isdir('world'):
    data_files.extend(package_dir_tree('world', f'share/{package_name}/world'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jao',
    maintainer_email='jaoaugusto@usp.br',
    description='projeto de ssc0712',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ground_truth_odometry = capote.ground_truth_odometry:main',
            'controle_robo = capote.controle_robo:main',
            'robo_mapper = capote.robo_mapper:main'
        ],
    },
)
