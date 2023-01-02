Run with artefacts:  
`colcon build --symlink-install && artefacts run nav_tests --nosim`

Run Blender script:  
`blender -b src/nav_test/blender/navigation.blend --python src/nav_test/blender/export_model.py`