from flask import Blueprint, render_template, jsonify, request
from routes.shared_state import *

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


@bp.route('/state')
def state():
    return jsonify(get_state())