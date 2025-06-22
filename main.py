from flask import Flask
from routes.routes import bp

app = Flask(__name__)
app.register_blueprint(bp)

if __name__ == '__main__':
    print("🎯 Avvio GUI Mirino Obsidian con controllo Joystick (via file condiviso)...")
    print("📍 Server disponibile su: http://localhost:7001")
    print("   • Ctrl+C per chiudere il server")

    app.run(host='0.0.0.0', port=7001, debug=False)
