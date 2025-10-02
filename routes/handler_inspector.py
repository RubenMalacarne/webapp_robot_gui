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
        
    @socketio.on('disconnect')
    def handle_disconnect():
        node = clients.pop(request.sid, 'unknown')
        log.info(f"Disconnessione nodo: {node} (sid: {request.sid})")
    
    @socketio.on('telemetry_data')
    def handle_telemetry(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"Telemetria ricevuta da {node}: {data}")
        
        # Rimanda i dati a tutti i client (es. la tua pagina inspector.html)
        socketio.emit('winch_telemetry', data, broadcast=True)