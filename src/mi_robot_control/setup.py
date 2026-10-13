from setuptools import find_packages, setup

package_name = 'mi_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel2404',
    maintainer_email='manuel.ortiz@eia.edu.co',
    description='Nodos de control cinemático (Line of Sight) para mi_robot_diferencial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # nombre_del_ejecutable = paquete.archivo:funcion
            'ir_a_pose = mi_robot_control.ir_a_pose:main',
            'control_los = mi_robot_control.control_los:main',
            'cinematica_llantas = mi_robot_control.cinematica_llantas:main'
        ],
    },
)
