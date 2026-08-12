from setuptools import setup

package_name = 'rl_ppo'

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
    maintainer='student',
    maintainer_email='@seas.upenn.edu',
    description='F1TENTH PPO Reinforcement Learning deployment package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ppo_deploy_node = rl_ppo.ppo_deploy_node:main',
        ],
    },
)
