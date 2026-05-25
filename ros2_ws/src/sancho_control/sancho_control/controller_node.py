import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # --- ROS 2 I/O ---
        # Ascolta i dati estratti da YOLO
        self.subscription = self.create_subscription(
            Point, 'target_tracking', self.tracking_callback, 10)
        
        # Invia i comandi motori al motor_bridge_node
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer di sicurezza (Watchdog a 10 Hz)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        # --- PARAMETRI DI CONTROLLO (Tuning) ---
        self.TARGET_DISTANCE = 1.5     # Distanza ideale in metri
        self.SAFE_DISTANCE = 1.2       # Sotto questa distanza, frenata d'emergenza
        self.MAX_DISTANCE = 3.5        # Oltre questa, velocità massima
        self.MAX_LINEAR_SPEED = 2.0    # m/s (La velocità di picco del tuo rover)
        self.MAX_ANGULAR_SPEED = 1.5   # rad/s (Velocità massima di rotazione)
        
        # --- STATO INTERNO E FILTRI ---
        self.last_msg_time = time.time()
        
        # Variabili per il filtro passa-basso (EMA - Exponential Moving Average)
        # Rendono i movimenti fluidi ignorando il "rumore" della telecamera
        self.smoothed_distance = None
        self.smoothed_offset = 0.0
        self.alpha = 0.2  # Quanto il filtro è "pesante" (0.1 = molto fluido/lento, 0.9 = molto reattivo/nervoso)

        self.get_logger().info("Nodo Controller avviato. In attesa del target...")

    def tracking_callback(self, msg):
        # Aggiorna il watchdog
        self.last_msg_time = time.time()

        raw_distance = msg.x
        raw_offset = msg.y

        # --- 1. APPLICAZIONE DEL FILTRO FLUIDO ---
        if self.smoothed_distance is None:
            self.smoothed_distance = raw_distance # Inizializzazione al primo colpo
        else:
            self.smoothed_distance = (self.alpha * raw_distance) + ((1.0 - self.alpha) * self.smoothed_distance)
            self.smoothed_offset = (self.alpha * raw_offset) + ((1.0 - self.alpha) * self.smoothed_offset)

        # --- 2. CALCOLO VELOCITÀ LINEARE (Avanti/Indietro) ---
        linear_x = 0.0
        
        if self.smoothed_distance <= self.SAFE_DISTANCE:
            linear_x = 0.0  # Troppo vicino: FERMO!
        elif self.smoothed_distance >= self.MAX_DISTANCE:
            linear_x = self.MAX_LINEAR_SPEED  # Molto lontano: VELOCITÀ MASSIMA!
        else:
            # Interpolazione lineare morbida: accelera proporzionalmente man mano che l'oggetto si allontana
            # Mappa la distanza [1.2 -> 3.5] alla velocità [0.0 -> 2.0]
            ratio = (self.smoothed_distance - self.SAFE_DISTANCE) / (self.MAX_DISTANCE - self.SAFE_DISTANCE)
            linear_x = ratio * self.MAX_LINEAR_SPEED

        # --- 3. CALCOLO VELOCITÀ ANGOLARE (Destra/Sinistra) ---
        # L'offset va da -1 (sinistra) a +1 (destra).
        # In ROS (REP-103), angular.z POSITIVO = gira a sinistra. 
        # Quindi se l'offset è +0.5 (target a destra), dobbiamo girare a destra (angular.z NEGATIVO).
        angular_z = -1.0 * self.smoothed_offset * self.MAX_ANGULAR_SPEED
        
        # --- 4. PUBBLICAZIONE DEL COMANDO MOTORI ---
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.publisher.publish(cmd)
        
        # Log per debug
        self.get_logger().debug(f"Dist: {self.smoothed_distance:.2f}m -> Vx: {linear_x:.2f} | Offset: {self.smoothed_offset:.2f} -> Wz: {angular_z:.2f}")

    def watchdog_check(self):
        # Se il nodo visione si blocca o perde il target per più di 0.5 secondi, ferma il rover.
        if time.time() - self.last_msg_time > 0.5:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            # Resetta la memoria del filtro per la prossima volta che trova il target
            self.smoothed_distance = None 

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Frena prima di spegnersi
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()