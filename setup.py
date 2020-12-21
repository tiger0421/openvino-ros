from setuptools import setup

package_name = 'vino'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        "scripts.segmentation"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openvino',
    maintainer_email='openvino@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "segmentation_image=scripts.segmentation:main",
        ],
    },
)
