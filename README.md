
Requirements:
 - ROS2 Humble
 - Gazebo Fortress https://gazebosim.org/docs/fortress/install_ubuntu
 - Artefacts https://docs.artefacts.com/latest/#getting-started-with-artefacts
 - Blender 3.4.1 or above

Run with artefacts:  
`colcon build --symlink-install && artefacts run nav_tests --nosim`

Run Blender script:  
`blender -b src/nav_test/blender/navigation.blend --python src/nav_test/blender/export_model.py`