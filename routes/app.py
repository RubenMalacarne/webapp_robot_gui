from flask import Blueprint, render_template, jsonify, request
from routes.shared_state import get_store

bp = Blueprint('main', __name__)

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