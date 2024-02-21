from setuptools import find_packages, setup

package_name = 'visual_servoing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','integ_my_robot_interfaces'],
    zip_safe=True,
    maintainer='t-lemmel',
    maintainer_email='lemmeltom@gmail.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'FeatureDetector = visual_servoing.FeatureDetector:main',
            'tracker = visual_servoing.tracker:main',
            'tracker2 = visual_servoing.tracker2:main',
        ],
    },
)