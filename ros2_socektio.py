#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socketio
import random
import time

class SensorDataSender(Node):

    def __init__(self):
        super().__init__('sensor_data_sender')

        # Configurazione Socket.IO (client)
        self.sio = socketio.Client()

        # Collegati al server Socket.IO
        self.sio.connect('http://localhost:7001')  # Cambia IP se server non locale

        self.get_logger().info("🌐 Connesso a Socket.IO")

        # Registra questo nodo
        self.sio.emit('register', {'node': self.get_name()})

        # Timer per temperatura ogni 2 secondi
        self.timer_temp = self.create_timer(2.0, self.send_temperature)
        self.timer_voltage = self.create_timer(0.1, self.send_random_voltage)
        self.timer_current = self.create_timer(0.2, self.send_random_current)
        # Contatore per simulazione
        self.time_counter = 0
    #test
    def send_temperature(self):
        temp = round(random.uniform(20.0, 30.0), 2)
        self.get_logger().info(f"🌡️ Inviando temperatura: {temp} °C")

        self.sio.emit('temperature', {
            'node': self.get_name(),
            'value': temp
        })
        
    def send_random_voltage(self):
        voltage = 12.0 + random.uniform(-1.0, 1.0)
        self.send_voltage_only(voltage)

    def send_random_current(self):
        current = 2.0 + random.uniform(-0.5, 0.5)
        self.send_current_only(current)
        

    # def send_sensor_data(self):
    #     """Invia dati di voltaggio e corrente"""
    #     import math
        
    #     # Genera dati realistici
    #     voltage = 12.0 + math.sin(self.time_counter * 0.1) * 2 + random.uniform(-0.5, 0.5)
    #     current = 2.0 + math.cos(self.time_counter * 0.15) * 0.8 + random.uniform(-0.2, 0.2)
        
    #     # Invia entrambi i valori insieme
    #     sensor_data = {
    #         'voltage': round(voltage, 2),
    #         'current': round(current, 2),
    #         'timestamp': int(time.time() * 1000),
    #         'node': self.get_name()
    #     }
        
    #     self.sio.emit('sensor_data', sensor_data)
        
    #     # Log ogni 50 campioni (ogni 5 secondi circa)
    #     if self.time_counter % 50 == 0:
    #         self.get_logger().info(f"📊 Dati sensori: {voltage:.2f}V, {current:.2f}A")
        
    #     self.time_counter += 1

    def send_voltage_only(self, voltage_value):
        """Metodo per inviare solo voltaggio (utile per sensori separati)"""
        voltage_data = {
            'value': round(voltage_value, 2),
            'timestamp': int(time.time() * 1000),
            'node': self.get_name()
        }
        self.sio.emit('voltage_data', voltage_data)
        
    def send_current_only(self, current_value):
        """Metodo per inviare solo corrente (utile per sensori separati)"""
        current_data = {
            'value': round(current_value, 2),
            'timestamp': int(time.time() * 1000),
            'node': self.get_name()
        }
        self.sio.emit('current_data', current_data)



def main(args=None):
    rclpy.init(args=args)
    node = SensorDataSender()  # Cambiato da TemperatureSender

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sio.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
