#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socketio
import time
from std_msgs.msg import Float32, Int32, Bool, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

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
            # Registra questo nodo
            self.sio.emit('register', {'node': 'inspector_gui'})
        except Exception as e:
            self.get_logger().error(f"❌ Impossibile connettersi a Socket.IO: {e}")
            raise
        
        # Dati del robot per l'inspector
        self.robot_status = {
            'esp32_status': False,
            'signal_strength': 0,
            'battery': 0,
            'battery_tank': 0
        }
        
        self.winch_status = {
            'winch_dx': {'connected': False, 'position': 0, 'speed': 0},
            'winch_sx': {'connected': False, 'position': 0, 'speed': 0}
        }
        
        self.control_values = {
            'winch_sx_values': [0, 0, 0],
            'winch_dx_values': [0, 0, 0]
        }
        
        # =========================== ROS2 Subscriptions ===========================
        
        # Robot Status Subscriptions
        self.esp32_status_sub = self.create_subscription(
            Bool, 'robot/esp32_status', self.esp32_status_callback, 10)
            
        self.signal_strength_sub = self.create_subscription(
            Int32, 'robot/signal_strength', self.signal_strength_callback, 10)
            
        self.battery_sub = self.create_subscription(
            BatteryState, 'robot/battery', self.battery_callback, 10)
            
        self.battery_tank_sub = self.create_subscription(
            Float32, 'robot/battery_tank', self.battery_tank_callback, 10)
        
        # Winch Status Subscriptions
        self.winch_dx_connected_sub = self.create_subscription(
            Bool, 'winch/dx/connected', self.winch_dx_connected_callback, 10)
            
        self.winch_sx_connected_sub = self.create_subscription(
            Bool, 'winch/sx/connected', self.winch_sx_connected_callback, 10)
            
        self.winch_dx_position_sub = self.create_subscription(
            Float32, 'winch/dx/position', self.winch_dx_position_callback, 10)
            
        self.winch_sx_position_sub = self.create_subscription(
            Float32, 'winch/sx/position', self.winch_sx_position_callback, 10)
            
        self.winch_dx_speed_sub = self.create_subscription(
            Float32, 'winch/dx/speed', self.winch_dx_speed_callback, 10)
            
        self.winch_sx_speed_sub = self.create_subscription(
            Float32, 'winch/sx/speed', self.winch_sx_speed_callback, 10)
        
        # Control Values Subscriptions
        self.winch_sx_value1_sub = self.create_subscription(
            Int32, 'control/winch_sx/value1', self.winch_sx_value1_callback, 10)
            
        self.winch_sx_value2_sub = self.create_subscription(
            Int32, 'control/winch_sx/value2', self.winch_sx_value2_callback, 10)
            
        self.winch_sx_value3_sub = self.create_subscription(
            Int32, 'control/winch_sx/value3', self.winch_sx_value3_callback, 10)
            
        self.winch_dx_value1_sub = self.create_subscription(
            Int32, 'control/winch_dx/value1', self.winch_dx_value1_callback, 10)
            
        self.winch_dx_value2_sub = self.create_subscription(
            Int32, 'control/winch_dx/value2', self.winch_dx_value2_callback, 10)
            
        self.winch_dx_value3_sub = self.create_subscription(
            Int32, 'control/winch_dx/value3', self.winch_dx_value3_callback, 10)
        
        # Timer per inviare periodicamente i dati alla web app (2Hz)
        self.timer_send_data = self.create_timer(0.5, self.send_all_data_periodic)
        
        self.get_logger().info("🔧 Inspector Data Sender inizializzato")
    
    # =========================== Robot Status Callbacks ===========================
    
    def esp32_status_callback(self, msg):
        """Callback per stato ESP32"""
        self.robot_status['esp32_status'] = msg.data
        self.send_robot_status()
        
    def signal_strength_callback(self, msg):
        """Callback per forza del segnale (0-100)"""
        self.robot_status['signal_strength'] = max(0, min(100, msg.data))
        self.send_robot_status()
        
    def battery_callback(self, msg):
        """Callback per batteria principale (BatteryState)"""
        # Converte da voltage a percentuale (esempio: 3.3V-4.2V -> 0-100%)
        voltage = msg.voltage
        percentage = max(0, min(100, ((voltage - 3.3) / (4.2 - 3.3)) * 100))
        self.robot_status['battery'] = int(percentage)
        self.send_robot_status()
        
    def battery_tank_callback(self, msg):
        """Callback per batteria tank (Float32 in percentuale)"""
        self.robot_status['battery_tank'] = max(0, min(100, int(msg.data)))
        self.send_robot_status()
    
    # =========================== Winch Status Callbacks ===========================
    
    def winch_dx_connected_callback(self, msg):
        """Callback per connessione winch destro"""
        self.winch_status['winch_dx']['connected'] = msg.data
        self.send_winch_status()
        
    def winch_sx_connected_callback(self, msg):
        """Callback per connessione winch sinistro"""
        self.winch_status['winch_sx']['connected'] = msg.data
        self.send_winch_status()
        
    def winch_dx_position_callback(self, msg):
        """Callback per posizione winch destro"""
        self.winch_status['winch_dx']['position'] = int(msg.data)
        self.send_winch_status()
        
    def winch_sx_position_callback(self, msg):
        """Callback per posizione winch sinistro"""
        self.winch_status['winch_sx']['position'] = int(msg.data)
        self.send_winch_status()
        
    def winch_dx_speed_callback(self, msg):
        """Callback per velocità winch destro"""
        self.winch_status['winch_dx']['speed'] = int(msg.data)
        self.send_winch_status()
        
    def winch_sx_speed_callback(self, msg):
        """Callback per velocità winch sinistro"""
        self.winch_status['winch_sx']['speed'] = int(msg.data)
        self.send_winch_status()
    
    # =========================== Control Values Callbacks ===========================
    
    def winch_sx_value1_callback(self, msg):
        """Callback per valore di controllo 1 winch sinistro"""
        self.control_values['winch_sx_values'][0] = msg.data
        self.send_control_values()
        
    def winch_sx_value2_callback(self, msg):
        """Callback per valore di controllo 2 winch sinistro"""
        self.control_values['winch_sx_values'][1] = msg.data
        self.send_control_values()
        
    def winch_sx_value3_callback(self, msg):
        """Callback per valore di controllo 3 winch sinistro"""
        self.control_values['winch_sx_values'][2] = msg.data
        self.send_control_values()
        
    def winch_dx_value1_callback(self, msg):
        """Callback per valore di controllo 1 winch destro"""
        self.control_values['winch_dx_values'][0] = msg.data
        self.send_control_values()
        
    def winch_dx_value2_callback(self, msg):
        """Callback per valore di controllo 2 winch destro"""
        self.control_values['winch_dx_values'][1] = msg.data
        self.send_control_values()
        
    def winch_dx_value3_callback(self, msg):
        """Callback per valore di controllo 3 winch destro"""
        self.control_values['winch_dx_values'][2] = msg.data
        self.send_control_values()
    
    # =========================== Socket.IO Send Functions ===========================
    
    def send_robot_status(self):
        """Invia stato robot alla web app"""
        if self.sio.connected:
            data = self.robot_status.copy()
            data['timestamp'] = time.time()
            self.sio.emit('robot_status', data)
            self.get_logger().debug(f"📡 Robot status inviato: {data}")
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare robot status")
    
    def send_winch_status(self):
        """Invia stato winch alla web app"""
        if self.sio.connected:
            data = self.winch_status.copy()
            data['timestamp'] = time.time()
            self.sio.emit('winch_status', data)
            self.get_logger().debug(f"🔧 Winch status inviato: {data}")
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare winch status")
    
    def send_control_values(self):
        """Invia valori di controllo alla web app"""
        if self.sio.connected:
            data = self.control_values.copy()
            data['timestamp'] = time.time()
            self.sio.emit('control_values', data)
            self.get_logger().debug(f"🎛️ Control values inviati: {data}")
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare control values")
    
    def send_all_data_periodic(self):
        """Invia tutti i dati periodicamente (chiamato dal timer)"""
        if self.sio.connected:
            self.send_robot_status()
            self.send_winch_status()
            self.send_control_values()
        else:
            self.get_logger().warn("Non connesso a Socket.IO per invio periodico")
    
    def destroy_node(self):
        """Cleanup quando il nodo viene distrutto"""
        self.get_logger().info("🔌 Inspector disconnettendo da Socket.IO...")
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectorDataSender()

    try:
        node.get_logger().info("🚀 Inspector Data Sender avviato!")
        node.get_logger().info("📋 Sottoscrizioni ROS2 attive:")
        node.get_logger().info("   Robot Status: esp32_status, signal_strength, battery, battery_tank")
        node.get_logger().info("   Winch Status: winch/dx|sx/connected, position, speed")
        node.get_logger().info("   Control Values: control/winch_dx|sx/value1|2|3")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interruzione richiesta dall'utente")
    except Exception as e:
        node.get_logger().error(f"❌ Errore: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("🔚 Inspector ROS2 node terminato")

if __name__ == '__main__':
    main()


# Esempi di comandi per testare i topic:
# 
# Robot Status:
# ros2 topic pub /robot/esp32_status std_msgs/msg/Bool "{data: true}" --once
# ros2 topic pub /robot/signal_strength std_msgs/msg/Int32 "{data: 95}" --once
# ros2 topic pub /robot/battery sensor_msgs/msg/BatteryState "{voltage: 3.8}" --once
# ros2 topic pub /robot/battery_tank std_msgs/msg/Float32 "{data: 85.5}" --once
#
# Winch Status:
# ros2 topic pub /winch/dx/connected std_msgs/msg/Bool "{data: true}" --once
# ros2 topic pub /winch/sx/connected std_msgs/msg/Bool "{data: true}" --once
# ros2 topic pub /winch/dx/position std_msgs/msg/Float32 "{data: 1500.0}" --once
# ros2 topic pub /winch/sx/position std_msgs/msg/Float32 "{data: 800.0}" --once
# ros2 topic pub /winch/dx/speed std_msgs/msg/Float32 "{data: 50.0}" --once
# ros2 topic pub /winch/sx/speed std_msgs/msg/Float32 "{data: -25.0}" --once
#
# Control Values:
# ros2 topic pub /control/winch_sx/value1 std_msgs/msg/Int32 "{data: 1024}" --once
# ros2 topic pub /control/winch_sx/value2 std_msgs/msg/Int32 "{data: 512}" --once
# ros2 topic pub /control/winch_sx/value3 std_msgs/msg/Int32 "{data: 256}" --once
# ros2 topic pub /control/winch_dx/value1 std_msgs/msg/Int32 "{data: 2048}" --once
# ros2 topic pub /control/winch_dx/value2 std_msgs/msg/Int32 "{data: 1024}" --once
# ros2 topic pub /control/winch_dx/value3 std_msgs/msg/Int32 "{data: 512}" --once