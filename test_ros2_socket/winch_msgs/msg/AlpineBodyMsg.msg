# Alpine Body Telemetry Message
# Contains IMU data from both rope and body sensors

# Rope IMU data
geometry_msgs/Quaternion rope_imu_orientation
geometry_msgs/Vector3 rope_imu_angular_velocity  
geometry_msgs/Vector3 rope_imu_rpy
geometry_msgs/Vector3 rope_imu_rpy_d
geometry_msgs/Vector3 rope_imu_acceleration

# Body IMU data  
geometry_msgs/Quaternion body_imu_orientation
geometry_msgs/Vector3 body_imu_angular_velocity
geometry_msgs/Vector3 body_imu_rpy
geometry_msgs/Vector3 body_imu_rpy_derivatives
geometry_msgs/Vector3 body_imu_acceleration