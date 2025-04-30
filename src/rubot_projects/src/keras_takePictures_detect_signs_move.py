#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import time
from datetime import datetime
import os

class KerasImageClassifier:
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

            # Publicar mensaje Twist basado en la clase detectada
            twist_msg = Twist()
            if class_name == "stop":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            elif class_name == "adelante":
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0
            elif class_name == "izquierda":
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = 0.3
            elif class_name == "derecha":
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -0.3
            else:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
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
    rospy.init_node("keras_detector")
    KerasImageClassifier()
    rospy.spin()