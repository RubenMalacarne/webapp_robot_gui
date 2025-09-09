import eventlet
eventlet.monkey_patch()

from flask import Flask
from routes.app import bp, socketio

app = Flask(__name__)
app.register_blueprint(bp)
socketio.init_app(app)
if __name__ == '__main__':
    print("🎯 Avvio GUI Mirino Obsidian con controllo Joystick (via file condiviso)...")
    print("📍 Server disponibile su: http://localhost:7001")
    print("   • Ctrl+C per chiudere il server")

    socketio.run(app, host='0.0.0.0', port=7001, debug=False)
