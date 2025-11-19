from setuptools import setup, find_packages

package_name = 'python_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='incheol',
    maintainer_email='you@example.com',
    description=' helper tools (CSV record/play etc.)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'record_on_execute = python_tools.record_on_execute:main',
            'play_and_record_csv_trajectory = python_tools.play_and_record_csv_trajectory:main',
        ],
    },
)
