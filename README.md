WIP

Requirements:
 - Blender 3.4.1 or above
 - ROS2 Humble
 - Gazebo Fortress https://gazebosim.org/docs/fortress/install_ubuntu
 - Artefacts https://docs.artefacts.com/latest/#getting-started-with-artefacts

### Run Blender script separately:  
```
blender -b src/nav_test/blender/navigation.blend --python src/nav_test/blender/export_model.py
```  
This creates a model at src/nav_test/models/Field

### Run with Artefacts locally
Install ROS dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
```
Build the ros project:
```
colcon build --base-paths src --symlink-install
source install/setup.bash
```
Run the tests with Artefacts:
```
artefacts run nav_tests
```
Alternatively, you can run a test with the `test_launching` package
```
launch_test src/nav_test/test/test_navigation.launch.py
```
