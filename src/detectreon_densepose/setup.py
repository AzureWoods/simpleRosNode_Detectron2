from setuptools import setup

package_name = 'detectreon_densepose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zi Lin',
    maintainer_email='zilin2020@u.northwestern.edu',
    description='Receive images and change them to densepose images.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = detectreon_densepose.densepose:main'
        ],
    },
)
