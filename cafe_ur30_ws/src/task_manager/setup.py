from setuptools import setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Candidate',
    maintainer_email='candidate@example.com',
    description='Task-level state machine for the café pick-and-place system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager_node = task_manager.task_manager_node:main',
        ],
    },
)
