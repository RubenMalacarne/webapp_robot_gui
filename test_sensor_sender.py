#!/usr/bin/env python3
"""
Test script per inviare dati di voltaggio e corrente tramite Socket.IO
Questo script simula un sensore che invia dati al dashboard
"""

import socketio
import time
import math
import random
import logging

# Configurazione logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SensorDataSender:
    def __init__(self, server_url='http://localhost:7001'):
        self.sio = socketio.Client()
        self.server_url = server_url
        self.connected = False
        
        # Setup event handlers
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        
    def on_connect(self):
        logger.info("🌐 Connesso al server Socket.IO")
        self.connected = True
        # Registra questo client come sensore
        self.sio.emit('register', {'node': 'sensor_simulator'})
        
    def on_disconnect(self):
        logger.info("❌ Disconnesso dal server")
        self.connected = False
        
    def connect(self):
        """Connetti al server"""
        try:
            self.sio.connect(self.server_url)
            return True
        except Exception as e:
            logger.error(f"Errore connessione: {e}")
            return False
            
    def disconnect(self):
        """Disconnetti dal server"""
        if self.connected:
            self.sio.disconnect()
            
    # def send_sensor_data(self, voltage, current, timestamp=None):
    #     """Invia entrambi i valori insieme"""
    #     if not self.connected:
    #         return False
            
    #     data = {
    #         'voltage': voltage,
    #         'current': current,
    #         'timestamp': timestamp or int(time.time() * 1000)
    #     }
        
    #     try:
    #         self.sio.emit('sensor_data', data)
    #         logger.info(f"📊 Inviati: {voltage:.2f}V, {current:.2f}A")
    #         return True
    #     except Exception as e:
    #         logger.error(f"Errore invio dati: {e}")
    #        return False
            
    def send_voltage_data(self, voltage, timestamp=None):
        """Invia solo dati di voltaggio"""
        if not self.connected:
            return False
            
        data = {
            'value': voltage,
            'timestamp': timestamp or int(time.time() * 1000)
        }
        
        try:
            self.sio.emit('voltage_data', data)
            logger.info(f"⚡ Voltaggio: {voltage:.2f}V")
            return True
        except Exception as e:
            logger.error(f"Errore invio voltaggio: {e}")
            return False
            
    def send_current_data(self, current, timestamp=None):
        """Invia solo dati di corrente"""
        if not self.connected:
            return False
            
        data = {
            'value': current,
            'timestamp': timestamp or int(time.time() * 1000)
        }
        
        try:
            self.sio.emit('current_data', data)
            logger.info(f"🔌 Corrente: {current:.2f}A")
            return True
        except Exception as e:
            logger.error(f"Errore invio corrente: {e}")
            return False

def simulate_sensor_data():
    """Simula dati realistici di sensori"""
    sender = SensorDataSender()
    
    if not sender.connect():
        logger.error("Impossibile connettersi al server")
        return
        
    try:
        test_time = 0
        logger.info("🚀 Avvio simulazione sensori... (Ctrl+C per fermare)")
        
        while True:
            # Genera dati realistici
            # Voltaggio: 12V nominale con variazioni
            voltage = 12.0 + math.sin(test_time * 0.1) * 2 + random.uniform(-0.5, 0.5)
            
            # Corrente: 2A nominale con variazioni
            current = 2.0 + math.cos(test_time * 0.15) * 0.8 + random.uniform(-0.2, 0.2)
            
            # Invia entrambi i valori insieme
            # sender.send_sensor_data(voltage, current)
            
            # Oppure invia separatamente (decommentare se necessario)
            sender.send_voltage_data(voltage)
            sender.send_current_data(current)
            
            test_time += 1
            time.sleep(0.1)  # 10 Hz
            
    except KeyboardInterrupt:
        logger.info("🛑 Simulazione fermata dall'utente")
    except Exception as e:
        logger.error(f"Errore durante la simulazione: {e}")
    finally:
        sender.disconnect()

if __name__ == "__main__":
    simulate_sensor_data()
