from setuptools import find_packages, setup

package_name = 'mavsdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stagiaire',
    maintainer_email='raphael.gleize@etu.u-bordeaux.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mav_node = mavsdk.drone_takeoff_service:main'
        ],
    },
)
