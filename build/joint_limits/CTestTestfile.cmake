# CMake generated Testfile for 
# Source directory: /home/sr/sr_ws/src/controller_manager/joint_limits
# Build directory: /home/sr/sr_ws/build/joint_limits
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_joint_limits_rosparam.launch.py "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/sr/sr_ws/build/joint_limits/test_results/joint_limits/test_joint_limits_rosparam.launch.py.xunit.xml" "--package-name" "joint_limits" "--output-file" "/home/sr/sr_ws/build/joint_limits/launch_test/test_joint_limits_rosparam.launch.py.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/sr/sr_ws/src/controller_manager/joint_limits/test/joint_limits_rosparam.launch.py" "--junit-xml=/home/sr/sr_ws/build/joint_limits/test_results/joint_limits/test_joint_limits_rosparam.launch.py.xunit.xml" "--package-name=joint_limits")
set_tests_properties(test_joint_limits_rosparam.launch.py PROPERTIES  LABELS "launch_test" TIMEOUT "60" WORKING_DIRECTORY "/home/sr/sr_ws/build/joint_limits" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/sr/sr_ws/src/controller_manager/joint_limits/CMakeLists.txt;31;add_launch_test;/home/sr/sr_ws/src/controller_manager/joint_limits/CMakeLists.txt;0;")
subdirs("gtest")