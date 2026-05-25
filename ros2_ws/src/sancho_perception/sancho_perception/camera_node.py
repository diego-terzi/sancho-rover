import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # --- ROS 2 PUBLISHERS ---
        # 1. Publisher per i dati di navigazione (Motori)
        self.tracking_pub = self.create_publisher(Point, 'target_tracking', 10)
        
        # 2. Publisher per lo streaming video (RViz / PC Esterno)
        self.image_pub = self.create_publisher(Image, 'camera/annotated_image', 10)
        
        # Strumento essenziale per convertire immagini OpenCV in messaggi ROS
        self.bridge = CvBridge()
        
        # Timer a 10 Hz (0.1s)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- YOLO CONFIGURATION ---
        self.get_logger().info("Caricamento YOLOv8n (Solo CPU) in modalità Headless...")
        self.model = YOLO('yolov8n.pt')
        self.TARGET_CLASS_ID = 0
        self.REAL_PERSON_WIDTH_M = 0.50
        self.FOCAL_LENGTH_PX = 700.0
        
        # --- CAMERA CONFIGURATION ---
        self.cap = cv2.VideoCapture(0)
        self.frame_width = 640
        self.frame_height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        
        self.get_logger().info("Nodo Visione Docker-Ready avviato e in ascolto.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Errore: impossibile leggere dalla telecamera USB!")
            return

        start_time = time.time()

        # Inference rimpicciolita
        results = self.model(frame, imgsz=128, classes=[self.TARGET_CLASS_ID], verbose=False)
        annotated_frame = frame.copy()
        
        if len(results[0].boxes) > 0:
            # Puntiamo alla PRIMA persona trovata
            box = results[0].boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            pixel_width = x2 - x1
            
            if pixel_width > 0:
                # 1. Matematica: Distanza e Offset
                distance_m = (self.REAL_PERSON_WIDTH_M * self.FOCAL_LENGTH_PX) / pixel_width
                person_center_x = (x1 + x2) / 2.0
                image_center_x = self.frame_width / 2.0
                offset = (person_center_x - image_center_x) / image_center_x
                
                # 2. Pubblicazione Dati Tracking (Per il nodo di controllo)
                msg_point = Point()
                msg_point.x = float(distance_m)
                msg_point.y = float(offset)
                msg_point.z = 0.0
                self.tracking_pub.publish(msg_point)
                
                # 3. Disegno sull'immagine per lo streaming
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                text = f"Dist: {distance_m:.2f}m | Offset: {offset:.2f}"
                cv2.putText(annotated_frame, text, (x1, max(20, y1 - 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Calcolo FPS interni per debug visivo
        fps = 1.0 / (time.time() - start_time)
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 4. Pubblicazione dello Streaming Video (Per RViz2)
        # Convertiamo il frame OpenCV in un messaggio ROS Image (bgr8 è lo standard dei colori OpenCV)
        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Errore nella conversione dell'immagine: {e}")

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().info("Arresto manuale richiesto...")
    finally:
        # Nessun destroyAllWindows da gestire!
        vision_node.cap.release()
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()