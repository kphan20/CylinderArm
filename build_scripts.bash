rosdep install -i --from-path src
colcon build --symlink-install --cmake-args # Allows for symbolic links to be used rather than compiling every time (useful for python)
# other options: --packages-up-to [PACKAGE}
source install/setup.bash