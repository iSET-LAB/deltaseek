import os

from setuptools import find_packages, setup


package_name = 'deltaseek_gazebo'


def package_files(directory):
    files = []
    for path, _, filenames in os.walk(directory):
        if '__pycache__' in path.split(os.sep):
            continue
        filenames = [
            name for name in filenames
            if not name.endswith(('.pyc', '.orig'))
        ]
        if not filenames:
            continue
        install_path = os.path.join('share', package_name, path)
        files.append((install_path, [os.path.join(path, name) for name in filenames]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + package_files('launch') + package_files('config') + package_files('urdf')
    + package_files('worlds'),
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='DeltaSeek project',
    maintainer_email='sxa4756@example.com',
    description='IFC-derived Gazebo benchmarks for a Clearpath A300 with UR5e.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_publisher = deltaseek_gazebo.ground_truth_publisher:main',
            'generate_benchmark = deltaseek_gazebo.generate_benchmark:main',
            'ifc_to_manifest = deltaseek_gazebo.ifc_to_manifest:main',
            'sample_deviations = deltaseek_gazebo.deviation_sampler:main',
            'evaluate_run = deltaseek_gazebo.evaluate:main',
            'compare_planners = deltaseek_gazebo.compare:main',
        ],
    },
)

