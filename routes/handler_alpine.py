from flask_socketio import SocketIO
import logging
from flask import request

log = logging.getLogger(__name__)

clients = {}

def init_button_handler(socketio: SocketIO):
    @socketio.on('register')
    def handle_register(data):
        node = data.get('node', 'unknown')
        clients[request.sid] = node
        #log.info(f"Nodo registrato: {node} (sid: {request.sid})")

    @socketio.on('disconnect')
    def handle_disconnect():
        node = clients.pop(request.sid, 'unknown')
        #log.info(f"Disconnessione nodo: {node} (sid: {request.sid})")

    # Gestione pulsanti di controllo
    @socketio.on('button_start_stop')
    def handle_start_stop(data):
        node = clients.get(request.sid, 'unknown')
        is_started = data.get('is_started', False)
        #log.info(f"Start/Stop da {node}: {'Started' if is_started else 'Stopped'}")
        socketio.emit('button_start_stop', {'is_started': is_started}, broadcast=True)

    @socketio.on('button_calibration')
    def handle_calibration(data):
        node = clients.get(request.sid, 'unknown')
        #log.info(f"Calibration richiesta da {node}")
        socketio.emit('button_calibration', {'triggered': True}, broadcast=True)

    @socketio.on('button_initialization')
    def handle_initialization(data):
        node = clients.get(request.sid, 'unknown')
        #log.info(f"Initialization richiesta da {node}")
        socketio.emit('button_initialization', {'triggered': True}, broadcast=True)

    # Gestione pulsanti di test
    @socketio.on('button_test_single_jump')
    def handle_test_single_jump(data):
        node = clients.get(request.sid, 'unknown')
        is_active = data.get('is_active', False)
        #log.info(f"Test Single Jump da {node}: {'Attivato' if is_active else 'Disattivato'}")
        socketio.emit('button_test_single_jump', {'is_active': is_active}, broadcast=True)

    @socketio.on('button_test_multi_jump')
    def handle_test_multi_jump(data):
        node = clients.get(request.sid, 'unknown')
        is_active = data.get('is_active', False)
        #log.info(f"Test Multi Jump da {node}: {'Attivato' if is_active else 'Disattivato'}")
        socketio.emit('button_test_multi_jump', {'is_active': is_active}, broadcast=True)

    @socketio.on('button_discrete_jump')
    def handle_discrete_jump(data):
        node = clients.get(request.sid, 'unknown')
        is_active = data.get('is_active', False)
        #log.info(f"Discrete Jump da {node}: {'Attivato' if is_active else 'Disattivato'}")
        socketio.emit('button_discrete_jump', {'is_active': is_active}, broadcast=True)

    @socketio.on('button_opti_jump')
    def handle_opti_jump(data):
        node = clients.get(request.sid, 'unknown')
        is_active = data.get('is_active', False)
        #log.info(f"Opti Jump da {node}: {'Attivato' if is_active else 'Disattivato'}")
        socketio.emit('button_opti_jump', {'is_active': is_active}, broadcast=True)

    # Gestione comandi (throttle/yaw) da ROS2
    @socketio.on('robot_cmd')
    def handle_robot_cmd(data):
        node = clients.get(request.sid, 'unknown')
        throttle = data.get('throttle', 0.0)
        yaw = data.get('yaw', 0.0)
        #log.info(f"Robot cmd da {node}: Throttle={throttle:.3f}, Yaw={yaw:.3f}")
        socketio.emit('robot_cmd', {
            'throttle': throttle,
            'yaw': yaw
        }, broadcast=True)
    
    # Gestione jump indicator
    @socketio.on('jump_indicator')
    def handle_jump_indicator(data):
        node = clients.get(request.sid, 'unknown')
        is_jumping = data.get('jumping', False)
        #log.info(f"Jump indicator da {node}: {'JUMPING' if is_jumping else 'not jumping'}")
        socketio.emit('jump_indicator', {
            'jumping': is_jumping
        }, broadcast=True)

    # Gestione salti rimanenti
    @socketio.on('jumps_remaining')
    def handle_jumps_remaining(data):
        node = clients.get(request.sid, 'unknown')
        total = data.get('total', 10)
        remaining = data.get('remaining', 10)
        #log.info(f"Salti rimanenti da {node}: {remaining}/{total}")
        socketio.emit('jumps_remaining', {
            'total': total,
            'remaining': remaining
        }, broadcast=True) 