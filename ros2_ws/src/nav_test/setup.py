from setuptools import setup
import os

package_name = "nav_test"


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

# Add folders recursively to data_files
def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=package_files(
        data_files, ["launch/", "models/", "rviz/", "worlds/"]
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="azazdeaz",
    maintainer_email="azazdeaz@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'pose_broadcaster = nav_test.pose_broadcaster:main',
            'robot = nav_test.robot:main',
        ],
    },
)
