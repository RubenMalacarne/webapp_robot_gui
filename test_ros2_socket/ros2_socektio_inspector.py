#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socketio
import time

from winch_msgs.msg import RopeTelemetry  
from winch_msgs.msg import AlpineBodyMsg
class InspectorDataSender(Node):

    def __init__(self):
        super().__init__('inspector_data_sender')

        # Configurazione Socket.IO (client)
        self.sio = socketio.Client()
        
        # Eventi di connessione Socket.IO
        @self.sio.event
        def connect():
            self.get_logger().info("🌐 Inspector connesso a Socket.IO server")
            
        @self.sio.event
        def disconnect():
            self.get_logger().warn("🔌 Inspector disconnesso da Socket.IO server")
            
        @self.sio.event
        def connect_error(data):
            self.get_logger().error(f"❌ Errore connessione Socket.IO: {data}")

        # Collegati al server Socket.IO
        try:
            self.sio.connect('http://localhost:7001')  # Cambia IP se server non locale
            self.get_logger().info("🌐 Inspector connesso a Socket.IO")
            # Registra questo nodo come ROS2 data sender, non come inspector GUI
            self.sio.emit('register', {'node': 'ros2_data_sender'})
        except Exception as e:
            self.get_logger().error(f"❌ Impossibile connettersi a Socket.IO: {e}")
            raise
        
        # Dati del robot per l'inspector
        self.winch_left_telemetry_data = {
            'rope_force': 0.0,
            'rope_length': 0.0,
            'rope_velocity': 0.0,
            'current': 0.0,
            'brake_status': False,
            'timestamp': 0.0
        }
        
        self.winch_right_telemetry_data = {
            'rope_force': 0.0,
            'rope_length': 0.0,
            'rope_velocity': 0.0,
            'current': 0.0,
            'brake_status': False,
            'timestamp': 0.0
        }
        
        # Alpine Body telemetry data
        self.alpine_body_data = {
            'rope_imu_orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'rope_imu_angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rope_imu_rpy': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rope_imu_rpy_d': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'rope_imu_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'body_imu_orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'body_imu_angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'body_imu_rpy': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'body_imu_rpy_derivatives': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'body_imu_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'timestamp': 0.0
        }
        
        
        # =========================== ROS2 Subscriptions ===========================
        
        self.winch_left_telemetry_sub = self.create_subscription(
            RopeTelemetry, '/winch/left/telemetry', self.winch_left_telemetry_callback, 10)
        
        self.winch_right_telemetry_sub = self.create_subscription(
            RopeTelemetry, '/winch/right/telemetry', self.winch_right_telemetry_callback, 10)
        
        self.alpine_body_sub = self.create_subscription(
            AlpineBodyMsg, '/alpine_body/telemetry', self.alpine_body_callback, 10)
        
        # Timer per invio periodico verso la web app (2 Hz)
        self.create_timer(0.5, self.send_periodic)
        
        self.get_logger().info("🔧 Inspector Data Sender inizializzato")
    
    # =========================== Robot Status Callbacks ===========================
    
    def winch_right_telemetry_callback(self, msg):
        """Callback per telemetry right"""
        self.winch_right_telemetry_data = {
            'rope_force': msg.rope_force,
            'rope_length': msg.rope_length,
            'rope_velocity': msg.rope_velocity,
            'current': msg.current,
            'brake_status': msg.brake_status,
            'timestamp': time.time()
        }
        self.get_logger().debug(f"📊 Winch Right Telemetry: force={msg.rope_force}, length={msg.rope_length}")
        
    def winch_left_telemetry_callback(self, msg):
        """Callback per telemetry left"""
        self.winch_left_telemetry_data = {
            'rope_force': msg.rope_force,
            'rope_length': msg.rope_length,
            'rope_velocity': msg.rope_velocity,
            'current': msg.current,
            'brake_status': msg.brake_status,
            'timestamp': time.time()
        }
        self.get_logger().debug(f"📊 Winch Left Telemetry: force={msg.rope_force}, length={msg.rope_length}")
    

    def alpine_body_callback(self, msg):
        """Callback per Alpine Body telemetry"""
        self.alpine_body_data = {
            'rope_imu_orientation': {
                'x': msg.rope_imu_orientation.x,
                'y': msg.rope_imu_orientation.y,
                'z': msg.rope_imu_orientation.z,
                'w': msg.rope_imu_orientation.w
            },
            'rope_imu_angular_velocity': {
                'x': msg.rope_imu_angular_velocity.x,
                'y': msg.rope_imu_angular_velocity.y,
                'z': msg.rope_imu_angular_velocity.z
            },
            'rope_imu_rpy': {
                'x': msg.rope_imu_rpy.x,
                'y': msg.rope_imu_rpy.y,
                'z': msg.rope_imu_rpy.z
            },
            'rope_imu_rpy_d': {
                'x': msg.rope_imu_rpy_d.x,
                'y': msg.rope_imu_rpy_d.y,
                'z': msg.rope_imu_rpy_d.z
            },
            'rope_imu_acceleration': {
                'x': msg.rope_imu_acceleration.x,
                'y': msg.rope_imu_acceleration.y,
                'z': msg.rope_imu_acceleration.z
            },
            'body_imu_orientation': {
                'x': msg.body_imu_orientation.x,
                'y': msg.body_imu_orientation.y,
                'z': msg.body_imu_orientation.z,
                'w': msg.body_imu_orientation.w
            },
            'body_imu_angular_velocity': {
                'x': msg.body_imu_angular_velocity.x,
                'y': msg.body_imu_angular_velocity.y,
                'z': msg.body_imu_angular_velocity.z
            },
            'body_imu_rpy': {
                'x': msg.body_imu_rpy.x,
                'y': msg.body_imu_rpy.y,
                'z': msg.body_imu_rpy.z
            },
            'body_imu_rpy_derivatives': {
                'x': msg.body_imu_rpy_derivatives.x,
                'y': msg.body_imu_rpy_derivatives.y,
                'z': msg.body_imu_rpy_derivatives.z
            },
            'body_imu_acceleration': {
                'x': msg.body_imu_acceleration.x,
                'y': msg.body_imu_acceleration.y,
                'z': msg.body_imu_acceleration.z
            },
            'timestamp': time.time()
        }
        self.get_logger().debug(f"📊 Alpine Body Telemetry received")

    # ----------- INVIO DATI A SOCKET.IO -----------
    def send_periodic(self):
        if not self.sio.connected:
            self.get_logger().warn_once("⚠️ Socket.IO non connesso, non invio telemetria")
            return

        data = {
            'winch_right': {**self.winch_right_telemetry_data, 'connected': True},
            'winch_left':  {**self.winch_left_telemetry_data,  'connected': True}
        }

        self.sio.emit('telemetry_data', data)
        self.sio.emit('alpine_body_telemetry', self.alpine_body_data)
        self.get_logger().debug(f"📡 Telemetria inviata: {data}")

    # ----------- CLEANUP -----------
    def destroy_node(self):
        self.get_logger().info("🔌 Chiusura connessione Socket.IO...")
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectorDataSender()

    try:
        node.get_logger().info("🚀 Inspector Data Sender avviato")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interruzione richiesta dall'utente")
    except Exception as e:
        node.get_logger().error(f"❌ Errore: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# Esempi di comandi per testare i topic con RopeTelemetry:
# 
# Test command for Alpine Body telemetry:
# ros2 topic pub /alpine_body/telemetry alpine_msgs/msg/AlpineBodyMsg "{rope_imu_orientation: {x: 0.1, y: 0.2, z: 0.3, w: 0.9}, rope_imu_angular_velocity: {x: 0.05, y: -0.03, z: 0.1}, rope_imu_rpy: {x: 0.1, y: 0.05, z: -0.02}, rope_imu_rpy_d: {x: 0.01, y: 0.005, z: -0.008}, rope_imu_acceleration: {x: 0.2, y: -0.1, z: 9.8}, body_imu_orientation: {x: 0.05, y: 0.1, z: 0.15, w: 0.95}, body_imu_angular_velocity: {x: 0.02, y: -0.01, z: 0.03}, body_imu_rpy: {x: 0.05, y: 0.02, z: -0.01}, body_imu_rpy_derivatives: {x: 0.005, y: 0.002, z: -0.001}, body_imu_acceleration: {x: 0.1, y: -0.05, z: 9.85}}" --once
#
# Winch Left Telemetry:
# ros2 topic pub /winch/left/telemetry winch_msgs/msg/RopeTelemetry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'winch_left'}, rope_force: 45.5, rope_length: 150.0, rope_velocity: 2.5, current: 8.3, brake_status: true}" --once
#
# Winch Right Telemetry:
# ros2 topic pub /winch/right/telemetry winch_msgs/msg/RopeTelemetry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'winch_right'}, rope_force: 52.1, rope_length: 200.0, rope_velocity: -1.8, current: 12.7, brake_status: false}" --once