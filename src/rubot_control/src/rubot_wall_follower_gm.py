#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class WallFollower:

    def __init__(self):
        """Inicializa las propiedades del nodo y configura los topics a publicar y suscribirse."""

        rospy.init_node("wall_follower_controller", anonymous=False)

        # Propiedades ingresadas con parametros
        self.__kp = rospy.get_param("~kp", 0)
        self.__Dr = rospy.get_param("~distance_reference", 0)
        self.__L = rospy.get_param("~lookahead_distance", 0)
        self.__theta = np.deg2rad(rospy.get_param("~theta", 0))
        self.__vForward = rospy.get_param("~forward_speed", 0)

        # Propiedades sin parametros
        self.__Df = 0
        self.__A = 0
        self.__B = 0

        # Estados disponibles (para mejorar legibilidad)
        self.__state_FOLLOWING_WALL = 0
        self.__state_APPROACHING_CORNER = 1
        self.__state_TUNRING = 2
        self.__state_APPROACHING_WALL = 3

        self.__state = self.__state_FOLLOWING_WALL

        # Propiedades secundarias

        # Tenemos operando dos versiones de Lidar que devuelven 360 0 720 puntos.
        # Para que el codigo sea compatible con cualquiera de los dos, aplicaremos
        # este factor de correccion en los angulos/indices de scan.ranges.
        # Se debe de calcular en la primera ejecucion de __callbackLaser(). Esta
        # variable sirve para asegurar que solo se ejecuta este calculo del
        # factor de correccion una sola vez.
        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 1

        # Definicion del mensaje
        self.__msg = Twist()
        self.__msg.linear.x = 0
        self.__msg.angular.z = 0

        # Configuracion del topic a publicar
        self.__cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Suscripcion a topic
        rospy.Subscriber("/scan", LaserScan, self.__callbackLaser)

        # Configuracions del metodo a ejecutar cuando se detenga el nodo
        rospy.on_shutdown(self.__shutdown)

    def start(self):
        """Inicia la operacion del nodo."""
        rospy.spin()

    def __callbackLaser(self, scan):
        """Funcion ejecutada cada vez que se recibe un mensaje en /scan."""

        # En la primera ejecucion, calculamos el factor de correcion
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(
                len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
        # wrong readings deliver 0 value in range
        """ newRange = []
        for val in scan.ranges:
            if (val == 0):
                newRange.append(3)
            else:
                newRange.append(val) """
        # Obtenemos las mediciones de A y B y las guardamos en variables locales
        """zero angle is on th back --> angle-180 """
        A = scan.ranges[int(round(self.__unWrapAngle(90 + np.rad2deg(self.__theta)))) * self.__scanRangesLengthCorrectionFactor]
        B = scan.ranges[(self.__unWrapAngle(90)) *
                        self.__scanRangesLengthCorrectionFactor]

        rospy.loginfo_once('Ángulo derecha:\t%d, Ángulo theta:\t%d',
            (self.__unWrapAngle(90)) * self.__scanRangesLengthCorrectionFactor,
            int(round(self.__unWrapAngle(90 + np.rad2deg(self.__theta)))) * self.__scanRangesLengthCorrectionFactor)
            
        rospy.loginfo("LIDAR: Delante:\t%.2f, Atrás:\t%.2f, Izquierda:\t%.2f, Derecha:\t%.2f, Theta:\t%.2f",
            scan.ranges[self.__unWrapAngle(180) * self.__scanRangesLengthCorrectionFactor],
            scan.ranges[self.__unWrapAngle(0) * self.__scanRangesLengthCorrectionFactor],
            scan.ranges[self.__unWrapAngle(-90) * self.__scanRangesLengthCorrectionFactor],
            scan.ranges[self.__unWrapAngle(90) * self.__scanRangesLengthCorrectionFactor],
            scan.ranges[int(round(self.__unWrapAngle(90 + np.rad2deg(self.__theta))) * self.__scanRangesLengthCorrectionFactor)])

        # Si el valor leido de A no es NaN ni infinito y da entre 0.1 y 16 m (limites del Lidar),
        # actualizamos la propiedad self.__A con el nuevo valor. wrong readings deliver 0 value in range
        if np.isfinite(A) and 0.1 <= A <= 16:
            self.__A = A

        # Idem con B
        if np.isfinite(B) and 0.1 <= B <= 16:
            self.__B = B

        # Calculamos el siguiente estado
        if not self.__isThereAWall(self.__A, 0.8) and \
           self.__state == self.__state_FOLLOWING_WALL:

            self.__state = self.__state_APPROACHING_CORNER

        elif not self.__isThereAWall(self.__B, 0.5) and \
                self.__state == self.__state_APPROACHING_CORNER:

            self.__state = self.__state_TUNRING

        elif self.__isThereAWall(self.__A, 0.3) and \
                self.__state == self.__state_TUNRING:

            self.__state = self.__state_APPROACHING_WALL

        elif self.__isThereAWall(self.__B, 0.5) and \
                self.__state == self.__state_APPROACHING_WALL:

            self.__state = self.__state_FOLLOWING_WALL

        # Actuamos en funcion del estado
        # Si estamos aproximandonos a la pared o a la esquina
        if self.__state in \
           {self.__state_APPROACHING_CORNER, self.__state_APPROACHING_WALL}:

            self.__msg.angular.z = 0
            self.__msg.linear.x = self.__vForward

        # Estamos girando
        elif self.__state == self.__state_TUNRING:

            self.__msg.angular.z = -self.__vForward / self.__Dr
            self.__msg.linear.x = self.__vForward

        elif self.__state == self.__state_FOLLOWING_WALL:

            # Calculamos el angulo alpha y limitamos su valor entre -90 y 90 grados
            alpha = np.clip(
                np.arctan(
                    (self.__A * np.cos(self.__theta) - self.__B) /
                    (self.__A * np.sin(self.__theta))),
                -np.pi/2, np.pi/2)

            # Calculamos la distancia actual
            Di = self.__B * np.cos(alpha)

            # Calculamos la distancia futura en una variable local
            Df = Di + self.__L * np.sin(alpha)

            # Si el valor calculado de Df no es NaN ni infinito y es mayor que 0,
            # actualizamos la propiedad self.__Df con el nuevo valor
            if Df >= 0 and np.isfinite(Df):
                self.__Df = Df

            # Control P
            # Calculamos el error
            error = self.__Dr - self.__Df

            # Calculamos la velocidad angular en una variable local
            angVel = self.__kp * error

            # Si el valor calculado de la velocidad angular no es NaN ni infinito,
            # actualizamos la propiedad self.__msg.angular.z con el nuevo valor
            if np.isfinite(angVel):
                self.__msg.angular.z = angVel

            # Ajustamos la velocidad lineal segun la velocidad angular
            if abs(self.__msg.angular.z) > 1:
                self.__msg.linear.x = 0
            elif abs(self.__msg.angular.z) > 0.03:
                self.__msg.linear.x = 0.5 * self.__vForward
            else:
                self.__msg.linear.x = self.__vForward

        # Publicamos el mensaje
        self.__cmdVel.publish(self.__msg)

    def __isThereAWall(self, valueMeasured, thresholdValue):
        """Indica si hay una pared en o no a una distancia indicada."""

        return valueMeasured < thresholdValue

    def __wrapAngle(self, angle):
        """Devuelve un angulo de -180 a 180."""

        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def __unWrapAngle(self, angle):
        """Devuelve un angulo de 0 a 360."""

        if angle < 0:
            return angle + 360
        else:
            return angle

    def __shutdown(self):
        """Para el robot antes de detener el nodo."""

        self.__msg.linear.x = 0
        self.__msg.angular.z = 0
        self.__cmdVel.publish(self.__msg)


if __name__ == '__main__':
    try:
        wf = WallFollower()
        wf.start()
    except rospy.ROSInterruptException:
        pass
