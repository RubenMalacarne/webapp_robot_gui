#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socketio
import random
import time

class TemperatureSender(Node):

    def __init__(self):
        super().__init__('temperature_sender')

        # Configurazione Socket.IO (client)
        self.sio = socketio.Client()

        # Collegati al server Socket.IO
        self.sio.connect('http://localhost:7001')  # Cambia IP se server non locale

        self.get_logger().info("🌐 Connesso a Socket.IO")

        # Timer per inviare dati ogni 2 secondi
        self.timer = self.create_timer(2.0, self.send_temperature)

    def send_temperature(self):
        temp = round(random.uniform(20.0, 30.0), 2)
        self.get_logger().info(f"🌡️ Inviando temperatura: {temp} °C")

        self.sio.emit('temperature', {
            'node': self.get_name(),
            'value': temp
        })
        #to know which node is sending the data
        self.sio.emit('register', {'node': self.get_name()})



def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSender()

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
