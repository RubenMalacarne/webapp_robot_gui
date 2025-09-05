from flask_socketio import SocketIO
import logging
from flask import request

log = logging.getLogger(__name__)

clients = {}

def init_temperature_handler(socketio: SocketIO):
    @socketio.on('register')
    def handle_register(data):
        node = data.get('node', 'unknown')
        clients[request.sid] = node
        log.info(f"Nodo registrato: {node} (sid: {request.sid})")

    @socketio.on('disconnect')
    def handle_disconnect():
        node = clients.pop(request.sid, 'unknown')
        log.info(f"Disconnessione nodo: {node} (sid: {request.sid})")


    @socketio.on('temperature')
    def handle_temperature(value):
        node = clients.get(request.sid, 'unknown')
        log.info(f"Ricevuto da {node}: {value} °C")
        socketio.emit('temperature', value, broadcast=True)

    @socketio.on('voltage_data')
    def handle_voltage(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"⚡ Voltaggio ricevuto da {node}: {data.get('value', 0)} V")
        socketio.emit('voltage_data', data, broadcast=True)

    @socketio.on('current_data')
    def handle_current(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"🔌 Corrente ricevuta da {node}: {data.get('value', 0)} A")
        socketio.emit('current_data', data, broadcast=True)