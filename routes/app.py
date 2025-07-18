from flask import Flask,Blueprint, render_template, jsonify, request
from routes.shared_state import get_store
from flask_socketio import SocketIO
import logging

import eventlet
eventlet.monkey_patch()   


app = Flask(__name__)
bp = Blueprint('main', __name__)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")

log = logging.getLogger(__name__)

socketio = SocketIO(
    cors_allowed_origins="*",
    async_mode="eventlet",
    logger=True,
    engineio_logger=False
)
clients = {}

@socketio.on('register')
def handle_register(data):
    node = data.get('node', 'unknown')
    clients[request.sid] = node
    log.info(f"📥 Nodo registrato: {node} (sid: {request.sid})")

@socketio.on('disconnect')
def handle_disconnect():
    node = clients.pop(request.sid, 'unknown')
    log.info(f"❌ Disconnessione nodo: {node} (sid: {request.sid})")
    
@socketio.on('temperature')
def handle_temperature(value):
    node = clients.get(request.sid, 'unknown')
    log.info(f"🌡️ Ricevuto da {node}: {value} °C")
    # Rimbalza a tutti i client web
    socketio.emit('temperature', value, broadcast=True)


@bp.route('/')
def index():
    return render_template('index.html')

@bp.route('/<page>')
def load_page(page):
    try:
        return render_template(page)
    except:
        return "Pagina non trovata", 404

@bp.route('/state/<name>', methods=['GET'])
def state(name):
    store = get_store(name)
    if store is None:
        return jsonify(error="state sconosciuto"), 404
    return jsonify(store.get())

@bp.route('/update/<name>', methods=['POST'])
def update(name):
    store = get_store(name)
    if store is None:
        return jsonify(error="state sconosciuto"), 404
    data = request.json or {}
    store.set(**data)
    return jsonify(success=True)