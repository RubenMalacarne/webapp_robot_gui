from flask_socketio import SocketIO, emit
import logging
from flask import request
import time
import random

log = logging.getLogger(__name__)

clients = {}

def init_button_handler(socketio: SocketIO):
    @socketio.on('register')
    def handle_register(data):
        node = data.get('node', 'unknown')
        clients[request.sid] = node
        log.info(f"Nodo registrato: {node} (sid: {request.sid})")
        
        # Invia dati iniziali se è l'inspector
        if node == 'inspector_gui':
            send_initial_data()

    @socketio.on('disconnect')
    def handle_disconnect():
        node = clients.pop(request.sid, 'unknown')
        log.info(f"Disconnessione nodo: {node} (sid: {request.sid})")
    
    @socketio.on('request_inspector_data')
    def handle_data_request(data):
        log.info(f"Richiesta dati da inspector: {data}")
        send_robot_status()
        send_winch_status()
        send_control_values()
    
    def send_initial_data():
        """Invia dati iniziali quando l'inspector si connette"""
        try:
            send_robot_status()
            send_winch_status()
            send_control_values()
            log.info("Dati iniziali inviati all'inspector")
        except Exception as e:
            log.error(f"Errore invio dati iniziali: {e}")
    
    def send_robot_status():
        """Invia stato del robot all'inspector"""
        try:
            # Simula dati del robot (sostituire con dati reali)
            robot_data = {
                'esp32_status': True,
                'signal_strength': random.randint(85, 100),
                'battery': random.randint(80, 100),
                'battery_tank': random.randint(90, 100),
                'timestamp': time.time()
            }
            
            # Invia solo ai client inspector
            for sid, node in clients.items():
                if node == 'inspector_gui':
                    socketio.emit('robot_status', robot_data, room=sid)
            
            log.debug(f"Status robot inviato: {robot_data}")
            
        except Exception as e:
            log.error(f"Errore invio status robot: {e}")
    
    def send_winch_status():
        """Invia stato dei winch all'inspector"""
        try:
            winch_data = {
                'winch_dx': {
                    'connected': True,
                    'position': random.randint(0, 1000),
                    'speed': random.randint(-100, 100)
                },
                'winch_sx': {
                    'connected': True,
                    'position': random.randint(0, 1000),
                    'speed': random.randint(-100, 100)
                },
                'timestamp': time.time()
            }
            
            # Invia solo ai client inspector
            for sid, node in clients.items():
                if node == 'inspector_gui':
                    socketio.emit('winch_status', winch_data, room=sid)
            
            log.debug(f"Status winch inviato: {winch_data}")
            
        except Exception as e:
            log.error(f"Errore invio status winch: {e}")
    
    def send_control_values():
        """Invia valori di controllo all'inspector"""
        try:
            control_data = {
                'winch_sx_values': [
                    random.randint(800, 1200),
                    random.randint(400, 600),
                    random.randint(200, 400)
                ],
                'winch_dx_values': [
                    random.randint(800, 1200),
                    random.randint(400, 600),
                    random.randint(200, 400)
                ],
                'timestamp': time.time()
            }
            
            # Invia solo ai client inspector
            for sid, node in clients.items():
                if node == 'inspector_gui':
                    socketio.emit('control_values', control_data, room=sid)
            
            log.debug(f"Valori controllo inviati: {control_data}")
            
        except Exception as e:
            log.error(f"Errore invio valori controllo: {e}")
    
    # Funzioni pubbliche per inviare dati da altre parti dell'applicazione
    def broadcast_robot_status(esp32_status=None, signal_strength=None, battery=None, battery_tank=None):
        """Funzione pubblica per inviare status robot da altre parti del codice"""
        robot_data = {
            'timestamp': time.time()
        }
        
        if esp32_status is not None:
            robot_data['esp32_status'] = esp32_status
        if signal_strength is not None:
            robot_data['signal_strength'] = signal_strength
        if battery is not None:
            robot_data['battery'] = battery
        if battery_tank is not None:
            robot_data['battery_tank'] = battery_tank
        
        for sid, node in clients.items():
            if node == 'inspector_gui':
                socketio.emit('robot_status', robot_data, room=sid)
        
        log.info(f"Robot status broadcast: {robot_data}")
    
    def broadcast_winch_status(winch_dx=None, winch_sx=None):
        """Funzione pubblica per inviare status winch da altre parti del codice"""
        winch_data = {
            'timestamp': time.time()
        }
        
        if winch_dx is not None:
            winch_data['winch_dx'] = winch_dx
        if winch_sx is not None:
            winch_data['winch_sx'] = winch_sx
        
        for sid, node in clients.items():
            if node == 'inspector_gui':
                socketio.emit('winch_status', winch_data, room=sid)
        
        log.info(f"Winch status broadcast: {winch_data}")
    
    def broadcast_control_values(winch_sx_values=None, winch_dx_values=None):
        """Funzione pubblica per inviare valori controllo da altre parti del codice"""
        control_data = {
            'timestamp': time.time()
        }
        
        if winch_sx_values is not None:
            control_data['winch_sx_values'] = winch_sx_values
        if winch_dx_values is not None:
            control_data['winch_dx_values'] = winch_dx_values
        
        for sid, node in clients.items():
            if node == 'inspector_gui':
                socketio.emit('control_values', control_data, room=sid)
        
        log.info(f"Control values broadcast: {control_data}")
    
    # Esponi le funzioni per uso esterno
    socketio.broadcast_robot_status = broadcast_robot_status
    socketio.broadcast_winch_status = broadcast_winch_status
    socketio.broadcast_control_values = broadcast_control_values
    
    # Background task per inviare dati periodici
    def start_periodic_data_updates():
        """Avvia task periodico per inviare dati aggiornati all'inspector"""
        def send_periodic_updates():
            while True:
                try:
                    time.sleep(2)  # Aggiorna ogni 2 secondi
                    
                    # Verifica se ci sono client inspector connessi
                    inspector_clients = [sid for sid, node in clients.items() if node == 'inspector_gui']
                    
                    if inspector_clients:
                        send_robot_status()
                        send_winch_status() 
                        send_control_values()
                        
                except Exception as e:
                    log.error(f"Errore nel task periodico: {e}")
                    time.sleep(5)  # Pausa più lunga in caso di errore
        
        # Avvia il task in background
        import threading
        update_thread = threading.Thread(target=send_periodic_updates, daemon=True)
        update_thread.start()
        log.info("Task periodico di aggiornamento dati avviato")
    
    # Avvia gli aggiornamenti periodici
    start_periodic_data_updates()
