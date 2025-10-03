from flask_socketio import SocketIO, emit
import logging
from flask import request
import time

log = logging.getLogger(__name__)

clients = {}

def init_button_handler(socketio: SocketIO):
    @socketio.on('register')
    def handle_register(data):
        node = data.get('node', 'unknown')
        clients[request.sid] = node
        log.info(f"✅ Nodo registrato: {node} (sid: {request.sid})")
        
    @socketio.on('disconnect')
    def handle_disconnect():
        node = clients.pop(request.sid, 'unknown')
        log.info(f"❌ Disconnessione nodo: {node} (sid: {request.sid})")
    
    # Handle Winch Telemetry (from ROS2 or test sender)
    @socketio.on('telemetry_data')
    def handle_telemetry(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"📊 Telemetria Winch ricevuta da {node}")
        
        # Broadcast winch telemetry to all clients (inspector + dashboard)
        socketio.emit('winch_telemetry', data, broadcast=True)
    
    # Handle direct winch_telemetry events
    @socketio.on('winch_telemetry')
    def handle_winch_telemetry_direct(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"📊 Telemetria Winch diretta da {node}")
        
        # Broadcast to all clients
        socketio.emit('winch_telemetry', data, broadcast=True)

    # Handle Alpine Body Telemetry (IMU data)
    @socketio.on('alpine_body_telemetry')
    def handle_alpine_body_telemetry(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"🎯 Telemetria Alpine Body ricevuta da {node}")
        
        # Broadcast Alpine Body data to all clients (inspector + dashboard)
        socketio.emit('alpine_body_telemetry', data, broadcast=True)
    
    # Handle data requests from inspector/dashboard
    @socketio.on('request_inspector_data')
    def handle_request_inspector_data(data):
        node = clients.get(request.sid, 'unknown')
        log.debug(f"📥 Richiesta dati inspector da {node}")
        # Data is handled by other nodes that respond to this request
    
    # Handle generic sensor data for backward compatibility
    @socketio.on('sensor_data')
    def handle_sensor_data(data):
        node = clients.get(request.sid, 'unknown')
        log.info(f"🔍 Dati sensore ricevuti da {node}: {data}")
        
        # Broadcast sensor data to all clients
        socketio.emit('sensor_data', data, broadcast=True)