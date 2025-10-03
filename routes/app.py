from flask import Flask,Blueprint, render_template, jsonify, request
from routes.shared_state import get_store
from flask_socketio import SocketIO
import logging

import eventlet
eventlet.monkey_patch()   

from .handler_alpine import init_button_handler
from .handler_inspector import init_button_handler as init_inspector_handler
from .handler_dashboard import init_button_handler as init_dashboard_handler

app = Flask(__name__)
bp = Blueprint('main', __name__)

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")

log = logging.getLogger(__name__)

socketio = SocketIO(
    cors_allowed_origins="*",
    async_mode="eventlet",
    logger=False,  # Disabilita i log dettagliati di SocketIO
    engineio_logger=False
)

# Initialize all handlers
init_button_handler(socketio)  # Alpine control buttons
init_inspector_handler(socketio)  # Inspector telemetry
init_dashboard_handler(socketio)  # Dashboard telemetry


@bp.route('/')
def index():
    return render_template('index.html')

@bp.route('/<page>')
def load_page(page):
    try:
        return render_template(page)
    except:
        return "Pagina non trovata", 404
    
# for POST requests to update shared state
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