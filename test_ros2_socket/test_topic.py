#!/usr/bin/env python3
"""
Test Topic Publisher for Dashboard
Publishes random telemetry data to ROS2 topics for testing the dashboard.
"""

import rclpy
from rclpy.node import Node
import random
import math
import time

from winch_msgs.msg import RopeTelemetry  
from winch_msgs.msg import AlpineBodyMsg
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header


class TelemetryTestPublisher(Node):
    """
    Publishes random telemetry data to simulate real robot behavior.
    Useful for testing the dashboard without actual hardware.
    """

    def __init__(self):
        super().__init__('telemetry_test_publisher')
        
        # Publishers for Winch telemetry
        self.winch_right_pub = self.create_publisher(
            RopeTelemetry, 
            '/winch/right/telemetry', 
            10
        )
        
        self.winch_left_pub = self.create_publisher(
            RopeTelemetry, 
            '/winch/left/telemetry', 
            10
        )
        
        # Publisher for Alpine Body telemetry
        self.alpine_body_pub = self.create_publisher(
            AlpineBodyMsg, 
            '/alpine_body/telemetry', 
            10
        )
        
        # Timer to publish at 10 Hz for smooth dashboard updates
        self.timer = self.create_timer(0.1, self.publish_telemetry)
        
        # Simulation time counter
        self.sim_time = 0.0
        
        # Base values for realistic simulation
        self.base_rope_force = 50.0
        self.base_rope_length = 150.0
        self.base_current = 8.0
        
        self.get_logger().info('🧪 Test Publisher inizializzato')
        self.get_logger().info('📊 Pubblicando su:')
        self.get_logger().info('   - /winch/right/telemetry')
        self.get_logger().info('   - /winch/left/telemetry')
        self.get_logger().info('   - /alpine_body/telemetry')
        self.get_logger().info('🚀 Frequenza: 10 Hz')
        self.get_logger().info('💡 Premi Ctrl+C per fermare')

    def generate_winch_telemetry(self, is_right=True):
        """Generate realistic random winch telemetry"""
        msg = RopeTelemetry()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'winch_right' if is_right else 'winch_left'
        
        # Add phase offset for left winch to create variation
        phase_offset = 0.0 if is_right else 0.5
        
        # Simulate rope force with oscillation + noise
        msg.rope_force = (
            self.base_rope_force + 
            20.0 * math.sin(self.sim_time * 0.5 + phase_offset) + 
            random.uniform(-3.0, 3.0)
        )
        
        # Simulate rope length with slow oscillation
        msg.rope_length = (
            self.base_rope_length + 
            30.0 * math.sin(self.sim_time * 0.3 + phase_offset) + 
            random.uniform(-2.0, 2.0)
        )
        
        # Simulate rope velocity (derivative of length oscillation)
        msg.rope_velocity = (
            2.0 * math.cos(self.sim_time * 0.5 + phase_offset) + 
            random.uniform(-0.5, 0.5)
        )
        
        # Simulate current correlated with force
        msg.current = (
            self.base_current + 
            3.0 * math.sin(self.sim_time * 0.7 + phase_offset) + 
            random.uniform(-0.5, 0.5)
        )
        
        # Random brake status (mostly false)
        msg.brake_status = random.random() < 0.05  # 5% chance of brake engaged
        
        return msg

    def generate_alpine_body_telemetry(self):
        """Generate realistic random Alpine Body IMU telemetry"""
        msg = AlpineBodyMsg()
        
        # Rope IMU - Orientation (Quaternion with small variations)
        msg.rope_imu_orientation = Quaternion()
        msg.rope_imu_orientation.x = 0.1 * math.sin(self.sim_time * 0.4) + random.uniform(-0.02, 0.02)
        msg.rope_imu_orientation.y = 0.1 * math.cos(self.sim_time * 0.4) + random.uniform(-0.02, 0.02)
        msg.rope_imu_orientation.z = 0.05 * math.sin(self.sim_time * 0.3) + random.uniform(-0.01, 0.01)
        msg.rope_imu_orientation.w = 0.99
        
        # Rope IMU - Angular Velocity
        msg.rope_imu_angular_velocity = Vector3()
        msg.rope_imu_angular_velocity.x = 0.05 * math.cos(self.sim_time * 0.8) + random.uniform(-0.02, 0.02)
        msg.rope_imu_angular_velocity.y = -0.03 * math.sin(self.sim_time * 0.9) + random.uniform(-0.02, 0.02)
        msg.rope_imu_angular_velocity.z = 0.04 * math.cos(self.sim_time * 0.7) + random.uniform(-0.02, 0.02)
        
        # Rope IMU - RPY (Roll, Pitch, Yaw)
        msg.rope_imu_rpy = Vector3()
        msg.rope_imu_rpy.x = 0.1 * math.sin(self.sim_time * 0.4) + random.uniform(-0.03, 0.03)
        msg.rope_imu_rpy.y = 0.05 * math.cos(self.sim_time * 0.5) + random.uniform(-0.02, 0.02)
        msg.rope_imu_rpy.z = -0.02 * math.sin(self.sim_time * 0.3) + random.uniform(-0.01, 0.01)
        
        # Rope IMU - RPY Derivatives
        msg.rope_imu_rpy_d = Vector3()
        msg.rope_imu_rpy_d.x = 0.01 * math.cos(self.sim_time * 0.8) + random.uniform(-0.005, 0.005)
        msg.rope_imu_rpy_d.y = 0.005 * math.sin(self.sim_time * 0.9) + random.uniform(-0.003, 0.003)
        msg.rope_imu_rpy_d.z = -0.008 * math.cos(self.sim_time * 0.7) + random.uniform(-0.004, 0.004)
        
        # Rope IMU - Acceleration
        msg.rope_imu_acceleration = Vector3()
        msg.rope_imu_acceleration.x = 0.2 * math.sin(self.sim_time * 0.6) + random.uniform(-0.1, 0.1)
        msg.rope_imu_acceleration.y = -0.1 * math.cos(self.sim_time * 0.7) + random.uniform(-0.05, 0.05)
        msg.rope_imu_acceleration.z = 9.8 + 0.3 * math.sin(self.sim_time * 0.5) + random.uniform(-0.15, 0.15)
        
        # Body IMU - Orientation (Quaternion)
        msg.body_imu_orientation = Quaternion()
        msg.body_imu_orientation.x = 0.05 * math.sin(self.sim_time * 0.3) + random.uniform(-0.01, 0.01)
        msg.body_imu_orientation.y = 0.08 * math.cos(self.sim_time * 0.35) + random.uniform(-0.02, 0.02)
        msg.body_imu_orientation.z = 0.03 * math.sin(self.sim_time * 0.25) + random.uniform(-0.01, 0.01)
        msg.body_imu_orientation.w = 0.99
        
        # Body IMU - Angular Velocity
        msg.body_imu_angular_velocity = Vector3()
        msg.body_imu_angular_velocity.x = 0.02 * math.cos(self.sim_time * 0.9) + random.uniform(-0.01, 0.01)
        msg.body_imu_angular_velocity.y = -0.01 * math.sin(self.sim_time * 1.0) + random.uniform(-0.01, 0.01)
        msg.body_imu_angular_velocity.z = 0.03 * math.cos(self.sim_time * 0.85) + random.uniform(-0.01, 0.01)
        
        # Body IMU - RPY
        msg.body_imu_rpy = Vector3()
        msg.body_imu_rpy.x = 0.05 * math.sin(self.sim_time * 0.3) + random.uniform(-0.02, 0.02)
        msg.body_imu_rpy.y = 0.02 * math.cos(self.sim_time * 0.4) + random.uniform(-0.01, 0.01)
        msg.body_imu_rpy.z = -0.01 * math.sin(self.sim_time * 0.25) + random.uniform(-0.01, 0.01)
        
        # Body IMU - RPY Derivatives
        msg.body_imu_rpy_derivatives = Vector3()
        msg.body_imu_rpy_derivatives.x = 0.005 * math.cos(self.sim_time * 0.9) + random.uniform(-0.002, 0.002)
        msg.body_imu_rpy_derivatives.y = 0.002 * math.sin(self.sim_time * 1.0) + random.uniform(-0.001, 0.001)
        msg.body_imu_rpy_derivatives.z = -0.001 * math.cos(self.sim_time * 0.85) + random.uniform(-0.001, 0.001)
        
        # Body IMU - Acceleration
        msg.body_imu_acceleration = Vector3()
        msg.body_imu_acceleration.x = 0.1 * math.sin(self.sim_time * 0.55) + random.uniform(-0.05, 0.05)
        msg.body_imu_acceleration.y = -0.05 * math.cos(self.sim_time * 0.65) + random.uniform(-0.03, 0.03)
        msg.body_imu_acceleration.z = 9.85 + 0.2 * math.sin(self.sim_time * 0.45) + random.uniform(-0.1, 0.1)
        
        return msg

    def publish_telemetry(self):
        """Publish all telemetry data"""
        
        # Generate and publish Winch Right telemetry
        winch_right_msg = self.generate_winch_telemetry(is_right=True)
        self.winch_right_pub.publish(winch_right_msg)
        
        # Generate and publish Winch Left telemetry
        winch_left_msg = self.generate_winch_telemetry(is_right=False)
        self.winch_left_pub.publish(winch_left_msg)
        
        # Generate and publish Alpine Body telemetry
        alpine_body_msg = self.generate_alpine_body_telemetry()
        self.alpine_body_pub.publish(alpine_body_msg)
        
        # Log summary every second (10 messages)
        if int(self.sim_time * 10) % 10 == 0:
            self.get_logger().info(
                f'📡 [{self.sim_time:.1f}s] '
                f'WR Force: {winch_right_msg.rope_force:.1f}N, '
                f'WL Force: {winch_left_msg.rope_force:.1f}N, '
                f'WR Length: {winch_right_msg.rope_length:.1f}m'
            )
        
        # Increment simulation time
        self.sim_time += 0.1


def main(args=None):
    rclpy.init(args=args)
    
    node = TelemetryTestPublisher()
    
    print('=' * 70)
    print('🧪 Test Telemetry Publisher for Dashboard')
    print('=' * 70)
    print('Pubblicando valori randomici sui topic ROS2...')
    print()
    print('📊 Topic pubblicati:')
    print('   • /winch/right/telemetry (RopeTelemetry)')
    print('   • /winch/left/telemetry (RopeTelemetry)')
    print('   • /alpine_body/telemetry (AlpineBodyMsg)')
    print()
    print('🎯 Frequenza: 10 Hz (aggiornamento ogni 0.1s)')
    print()
    print('💡 Assicurati che ros2_socektio_dashboard.py sia in esecuzione!')
    print('💡 Apri la dashboard su: http://localhost:7001/dashboard.html')
    print()
    print('🛑 Premi Ctrl+C per fermare')
    print('=' * 70)
    print()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Interruzione utente')
    except Exception as e:
        node.get_logger().error(f'❌ Errore: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('✅ Test publisher terminato')


if __name__ == '__main__':
    main()
