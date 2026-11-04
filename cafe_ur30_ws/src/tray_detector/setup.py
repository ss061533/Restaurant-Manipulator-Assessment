from setuptools import setup

package_name = 'tray_detector'

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
    description='RGB-D colour-based tray detector',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tray_detector_node = tray_detector.detector_node:main',
        ],
    },
)
