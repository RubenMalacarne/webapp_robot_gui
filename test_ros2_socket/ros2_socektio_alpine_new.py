#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socketio
import random
import time
from std_msgs.msg import Float32
from std_srvs.srv import Trigger  
class AlpineDataSender(Node):

    def __init__(self):
        super().__init__('alpine_data_sender')

        # Configurazione Socket.IO (client)
        self.sio = socketio.Client()
        
        # Eventi di connessione Socket.IO
        @self.sio.event
        def connect():
            self.get_logger().info("🌐 Connesso a Socket.IO server")
            
        @self.sio.event
        def disconnect():
            self.get_logger().warn("🔌 Disconnesso da Socket.IO server")
            
        @self.sio.event
        def connect_error(data):
            self.get_logger().error(f"❌ Errore connessione Socket.IO: {data}")

        # Collegati al server Socket.IO
        try:
            self.sio.connect('http://localhost:7001')  # Cambia IP se server non locale
            self.get_logger().info("🌐 Connesso a Socket.IO")
            # Registra questo nodo
            self.sio.emit('register', {'node': self.get_name()})
        except Exception as e:
            self.get_logger().error(f"❌ Impossibile connettersi a Socket.IO: {e}")
            raise
        
        # Contatore per simulazione
        self.time_counter = 0
        
        # Stato dei pulsanti ricevuti dalla web app
        self.button_states = {
            'start_stop': False,
            'calibration': False,
            'initialization': False,
            'test_single_jump': False,
            'test_multi_jump': False,
            'discrete_jump': False,
            'opti_jump': False,
        }
        
        # ROS2 Subscribers per ricevere dati joystick
        self.robot_cmd = {"throttle": 0.0, "yaw": 0.0}
        self.throttle_sub = self.create_subscription(
            Float32, 'joystick/thrust', self.throttle_callback, 10)
        self.yaw_sub = self.create_subscription(
            Float32, 'joystick/yaw', self.yaw_callback, 10)
        self.srv_trigger_jump = self.create_service(
            Trigger, 'joystick/SE', self.trigger_jump_service_callback
        )
        
        # Timer per inviare periodicamente i dati alla web app (10Hz)
        self.timer_robot_cmd = self.create_timer(0.1, self.send_robot_cmd_periodic)
        # Configura i listener per ricevere stati dei pulsanti
        self.setup_button_listeners()
        self.is_jumping = False
    
    def throttle_callback(self, msg):
        """Callback per ricevere dati throttle dal joystick ROS2"""
        self.robot_cmd["throttle"] = msg.data
        # Non inviamo subito, lasciamo che il timer gestisca l'invio
        
    def yaw_callback(self, msg):
        """Callback per ricevere dati yaw dal joystick ROS2"""
        self.robot_cmd["yaw"] = msg.data
        # Non inviamo subito, lasciamo che il timer gestisca l'invio
        
    def trigger_jump_service_callback(self, request, response):
        """Callback per il service Trigger joystick/SE"""
        self.get_logger().info("📡 Service 'joystick/SE' chiamato!")
        self.trigger_jump_indicator()
        response.success = True
        response.message = "Jump indicator attivato con successo"
        return response
        
    def trigger_jump_indicator(self):
        """Attiva il jump indicator nella web app"""
        #if is jumping was already true now do false o viceversa
        self.is_jumping = not self.is_jumping
        if self.sio.connected:
            self.sio.emit('jump_indicator', {'jumping': self.is_jumping})
            self.get_logger().info("🦘 Jump indicator inviato alla web app!")
            
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare jump indicator")
    
    def send_robot_cmd_periodic(self):
        """Invia periodicamente i dati robot alla web app (chiamato dal timer)"""
        if self.sio.connected:
            self.sio.emit("robot_cmd", self.robot_cmd)
            # Log solo se ci sono movimenti significativi
            if abs(self.robot_cmd["throttle"]) > 0.1 or abs(self.robot_cmd["yaw"]) > 0.1:
                self.get_logger().info(f"🕹️ Robot cmd: Throttle={self.robot_cmd['throttle']:.2f}, Yaw={self.robot_cmd['yaw']:.2f}")
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare comandi")
    
    def send_robot_cmd(self):
        """Invia immediatamente i dati robot alla web app (per compatibilità)"""
        if self.sio.connected:
            self.sio.emit("robot_cmd", self.robot_cmd)
        else:
            self.get_logger().warn("Non connesso a Socket.IO, impossibile inviare comandi")

    def setup_button_listeners(self):
        """Configura tutti i listener per ricevere lo stato dei pulsanti dalla web app"""
        
        @self.sio.on('button_start_stop')
        def on_start_stop(data):
            self.button_states['start_stop'] = data.get('is_started', False)
            self.get_logger().info(f"🔄 Start/Stop: {'Started' if self.button_states['start_stop'] else 'Stopped'}")
            
        @self.sio.on('button_calibration')
        def on_calibration(data):
            if data.get('triggered', False):
                self.button_states['calibration'] = True
                self.get_logger().info("🎯 Calibration richiesta!")
                self.handle_calibration()
                
        @self.sio.on('button_initialization')
        def on_initialization(data):
            if data.get('triggered', False):
                self.button_states['initialization'] = True
                self.get_logger().info("🚀 Initialization richiesta!")
                self.handle_initialization()
                
        @self.sio.on('button_test_single_jump')
        def on_test_single_jump(data):
            self.button_states['test_single_jump'] = data.get('is_active', False)
            if self.button_states['test_single_jump']:
                self.get_logger().info("🦘 Test Single Jump attivato")
                self.reset_other_test_modes('test_single_jump')
                
        @self.sio.on('button_test_multi_jump')
        def on_test_multi_jump(data):
            self.button_states['test_multi_jump'] = data.get('is_active', False)
            if self.button_states['test_multi_jump']:
                self.get_logger().info("🦘🦘 Test Multi Jump attivato")
                self.reset_other_test_modes('test_multi_jump')
                
        @self.sio.on('button_discrete_jump')
        def on_discrete_jump(data):
            self.button_states['discrete_jump'] = data.get('is_active', False)
            if self.button_states['discrete_jump']:
                self.get_logger().info("📐 Discrete Jump attivato")
                self.reset_other_test_modes('discrete_jump')
                
        @self.sio.on('button_opti_jump')
        def on_opti_jump(data):
            self.button_states['opti_jump'] = data.get('is_active', False)
            if self.button_states['opti_jump']:
                self.get_logger().info("⚡ Opti Jump attivato")
                self.reset_other_test_modes('opti_jump')
        
        # Listener opzionale per joystick_data (se serve supportare anche questo formato)
        @self.sio.on('joystick_data')
        def on_joystick_data(data):
            self.get_logger().info(f"🎮 Ricevuto joystick_data: {data}")
            # Puoi elaborare qui se ricevi dati joystick da altre fonti
                
    def reset_other_test_modes(self, active_mode):
        """Disattiva tutti gli altri test modes quando ne viene attivato uno nuovo"""
        test_modes = ['test_single_jump', 'test_multi_jump', 'discrete_jump', 'opti_jump']
        for mode in test_modes:
            if mode != active_mode:
                self.button_states[mode] = False
    
    def handle_calibration(self):
        """Gestisce la logica di calibrazione"""
        self.get_logger().info("Eseguendo calibrazione...")
        # Qui aggiungi la tua logica di calibrazione
        # Esempio: pubblicare su topic ROS2, chiamare servizi, etc.
        
    def handle_initialization(self):
        """Gestisce la logica di inizializzazione"""
        self.get_logger().info("Eseguendo inizializzazione...")
        self.trigger_jump_indicator()
        # Qui aggiungi la tua logica di inizializzazione
        
    def get_current_state(self):
        """Restituisce lo stato corrente di tutti i pulsanti"""
        return self.button_states.copy()
        
    def is_system_active(self):
        """Controlla se il sistema è attivo (start/stop)"""
        return self.button_states['start_stop']
        
    def get_active_test_mode(self):
        """Restituisce il test mode attualmente attivo"""
        test_modes = ['test_single_jump', 'test_multi_jump', 'discrete_jump', 'opti_jump']
        for mode in test_modes:
            if self.button_states[mode]:
                return mode
        return None
                
    def execute_jump(self, mode):
        """Esegue il salto basato sul modo selezionato"""
        if mode == 'test_single_jump':
            self.get_logger().info("Executing single jump")
            # Logica per single jump
            
        elif mode == 'test_multi_jump':
            self.get_logger().info("Executing multi jump")
            # Logica per multi jump
            
        elif mode == 'discrete_jump':
            self.get_logger().info("Executing discrete jump")
            # Logica per discrete jump
            
        elif mode == 'opti_jump':
            self.get_logger().info("Executing optimized jump")
            # Logica per optimized jump
    
    def destroy_node(self):
        """Cleanup quando il nodo viene distrutto"""
        self.get_logger().info("🔌 Disconnettendo da Socket.IO...")
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlpineDataSender()

    try:
        node.get_logger().info("🚀 AlpineDataSender avviato!")
        node.get_logger().info("📋 Stato iniziale pulsanti:")
        for key, value in node.get_current_state().items():
            node.get_logger().info(f"   {key}: {value}")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interruzione richiesta dall'utente")
    except Exception as e:
        node.get_logger().error(f"❌ Errore: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("🔚 ROS2 node terminato")

if __name__ == '__main__':
    main()
