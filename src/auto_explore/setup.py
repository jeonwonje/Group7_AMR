from setuptools import find_packages, setup

package_name = 'auto_explore'

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
    maintainer='kuga',
    maintainer_email='kugapryank@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'find_frontiers = auto_explore.find_frontiers:main',
            'score_and_post = auto_explore.score_and_post:main',
            'wander = auto_explore.wander:main',
        ],
    },
)
