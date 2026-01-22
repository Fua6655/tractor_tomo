from setuptools import setup

package_name = 'tomo_ps4'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luka',
    maintainer_email='luka@example.com',
    description='PS4 → ControlEvents publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ps4_node = tomo_ps4.ps4_node:main',
        ],
    },
)
