from setuptools import find_packages, setup

package_name = 'tramola'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy<2'],
    zip_safe=True,
    maintainer='efe',
    maintainer_email='efe@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'video = tramola.video:main',
        'followPath = tramola.followPath:main',
        'vision = tramola.vision:main',
        ],
    },
)
