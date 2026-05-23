from setuptools import setup

package_name = 'ponderada_vc'

setup(
    name=package_name,
    version='0.0.0',

    packages=['ponderada_vc', 'ponderada_vc.projeto'],
    package_data={
        'ponderada_vc': ['dog.jpg'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='cauex',
    maintainer_email='cauex@todo.todo',

    description='Pipeline de visao computacional',
    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'turtle_controller = ponderada_vc.projeto.turtle_controller:main',
        ],
    },
)

