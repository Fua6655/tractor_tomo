from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tomo_web'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(include=['tomo_web', 'tomo_web.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/html', glob('tomo_web/html/index.html')),
        ('share/' + package_name + '/html/css', glob('tomo_web/html/css/*.css')),
        ('share/' + package_name + '/html/js', glob('tomo_web/html/js/*.js')),
    ],
    install_requires=['setuptools'],
    package_data={
        'tomo_web': [
            'html/*',
            'html/css/*',
            'html/js/*',
            'html/fonts/*',
        ]
    },
    zip_safe=True,
    maintainer='luka',
    description='TOMO Web UI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_node = tomo_web.web_node:main',
            'web_server = tomo_web.web_server_old:main',
        ],
    },
)
