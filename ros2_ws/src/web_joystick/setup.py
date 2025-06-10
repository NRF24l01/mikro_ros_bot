from setuptools import find_packages, setup

package_name = 'web_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'Flask', 'Flask-SocketIO'],
    zip_safe=True,
    maintainer='technik12345',
    maintainer_email='s.nevrovsky@ya.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web = web_joystick.web_joystick:main',
        ],
    },
)
