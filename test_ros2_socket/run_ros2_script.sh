# Source ROS2 environment
source install/setup.bash

# Run the three Python files
python3 ros2_socektio_alpine.py &
python3 ros2_socektio_dashboard.py &
python3 ros2_socektio_inspector.py &
python3 test_topic.py

