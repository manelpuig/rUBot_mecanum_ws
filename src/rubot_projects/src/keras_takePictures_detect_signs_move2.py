#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import time
from datetime import datetime
import os
import tf_transformations

class KerasImageClassifier_move:
    def __init__(self):
        # Paths relativos al script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "../models/keras_model.h5")
        labels_path = os.path.join(script_dir, "../models/labels.txt")

        # Cargar modelo y etiquetas
        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)
        self.input_shape = self.model.input_shape[1:3]

        # Imagen
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.class_pub = rospy.Publisher("/predicted_class", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # Publisher para Twist

        # Odometria
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Posició del senyal detectat
        self.signal_x = None
        self.signal_y = None
        self.last_detection_time = None
        self.detection_timeout = 10.0 # Segons per considerar una detecció vàlida

        # Paràmetre de distància per actuar
        self.detection_distance_threshold = 0.4# Distancia en m a la senyal

        # Capturas
        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/rUBot_mecanum_ws/src/rubot_projects/rUBot_captures")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.create_class_dirs()  # Crear carpetas por clase

        self.last_capture_time = time.time()
        self.capture_interval = 1.0  # segundos

        # Control de captura por topic
        rospy.Subscriber("/capture_toggle", Bool, self.toggle_callback)

        rospy.loginfo("Nodo keras_detector activo. Esperando imágenes...")

    def load_labels(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()
            return [line.strip().split(' ', 1)[1] for line in lines]

    def create_class_dirs(self):
        for class_name in self.labels:
            class_path = os.path.join(self.capture_dir, class_name)
            os.makedirs(class_path, exist_ok=True)

    def toggle_callback(self, msg):
        self.capture_enabled = msg.data
        state = "ACTIVADA" if self.capture_enabled else "DESACTIVADA"
        rospy.loginfo(f"Captura automática {state}")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        try:
            # Convertir imagen
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            resized = cv2.resize(cv_image, self.input_shape)
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            # Predicción
            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]
            rospy.loginfo(f"Detectado: {class_name}")
            self.class_pub.publish(class_name)

            # Actualitzar posició del senyal (simplificació: assumim que el senyal està sempre davant del robot)
            self.signal_x = self.robot_x + 0.5 # Assumim que el senyal es detecta a 0.5m davant
            self.signal_y = self.robot_y
            self.last_detection_time = rospy.Time.now()

            # Calcular distància al senyal i actuar
            if self.signal_x is not None and self.signal_y is not None and self.last_detection_time is not None:
                time_diff = rospy.Time.now() - self.last_detection_time
                if time_diff.to_sec() < self.detection_timeout:
                    distance_to_signal = np.sqrt((self.robot_x - self.signal_x)**2 + (self.robot_y - self.signal_y)**2)
                    rospy.loginfo(f"Distancia al senyal {class_name}: {distance_to_signal:.2f} metres")

                    twist_msg = Twist()
                    if distance_to_signal <= self.detection_distance_threshold:
                        if class_name == "Stop":
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                        elif class_name == "Give_Way":
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0 # Aturar abans de passar
                        elif class_name == "Turn_Left":
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.5 # Girar més pronunciadament a l'esquerra
                        elif class_name == "Turn_Right":
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = -0.5 # Girar més pronunciadament a la dreta
                        self.cmd_vel_pub.publish(twist_msg)
            # Guardar imagen si está activado
            if self.capture_enabled:
                current_time = time.time()
                if current_time - self.last_capture_time >= self.capture_interval:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"{class_name}_{timestamp}.jpg"
                    class_folder = os.path.join(self.capture_dir, class_name)
                    filepath = os.path.join(class_folder, filename)
                    cv2.imwrite(filepath, cv_image)
                    rospy.loginfo(f"Imagen guardada: {filepath}")
                    self.last_capture_time = current_time

        except CvBridgeError as e:
            rospy.logerr(f"Error de CvBridge: {e}")
        except Exception as e:
            rospy.logerr(f"Error procesando imagen: {e}")

if __name__ == "__main__":
    rospy.init_node("keras_takePictures_detect_signs_move")
    KerasImageClassifier_move()
    rospy.spin()