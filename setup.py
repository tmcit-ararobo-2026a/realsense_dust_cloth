from setuptools import find_packages, setup

package_name = 'dust_cloth_subscriber'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Mop detection package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'コマンド名 = パッケージ名.ファイル名:関数名'
            'cloth_subscribe_node = dust_cloth_subscriber.cloth_subscribe_node:main'
        ],
    },
)