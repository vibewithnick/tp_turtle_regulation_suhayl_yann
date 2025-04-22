#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtle_interfaces.srv import SetWayPoint as SetWayPointSrv
import numpy as np

class SetWayPoint(Node):
    def __init__(self):
        super().__init__('set_way_point')
        
        # Définir un attribut du nom de waypoint avec pour coordonnées (7,7)
        self.waypoint = (7.0, 7.0)
        
        # Subscriber pour la pose de la tortue
        self.pose_subscription = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            10)
        
        # Publisher pour la commande de vitesse
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            
        # Publisher pour indiquer si la tortue est en mouvement
        self.is_moving_publisher = self.create_publisher(
            Bool,
            'is_moving',
            10)
            
        # Service pour modifier le waypoint
        self.set_waypoint_service = self.create_service(
            SetWayPointSrv,
            'set_waypoint_service',
            self.set_waypoint_callback)
        
        # Constantes pour la régulation
        self.Kp = 1.0  # Constante pour la régulation en cap
        self.Kpl = 1.0  # Constante pour la régulation en distance
        self.distance_tolerance = 0.1  # Seuil de tolérance pour la distance
        
        # Attributs pour stocker la pose courante
        self.current_pose = None
        
        self.get_logger().info('Set waypoint node has been initialized with waypoint: ({}, {})'.format(
            self.waypoint[0], self.waypoint[1]))

    def set_waypoint_callback(self, request, response):
        """Callback pour le service set_waypoint_service"""
        self.waypoint = (request.x, request.y)
        self.get_logger().info('Waypoint updated to: ({}, {})'.format(request.x, request.y))
        response.res = True
        return response

    def pose_callback(self, msg):
        """Callback pour le topic pose"""
        self.current_pose = msg
        self.regulate()
    
    def calculate_desired_angle(self):
        """Calcule l'angle désiré entre la tortue et le waypoint"""
        if self.current_pose is None:
            return None
        
        # Coordonnées du waypoint et de la tortue
        x_w, y_w = self.waypoint
        x_t, y_t = self.current_pose.x, self.current_pose.y
        
        # Calcul de l'angle désiré en utilisant la formule θ_desired = atan2(y_w - y_t, x_w - x_t)
        theta_desired = math.atan2(y_w - y_t, x_w - x_t)
        
        return theta_desired
    
    def calculate_linear_error(self):
        """Calcule la distance euclidienne entre la tortue et le waypoint"""
        if self.current_pose is None:
            return None
        
        # Coordonnées du waypoint et de la tortue
        x_w, y_w = self.waypoint
        x_t, y_t = self.current_pose.x, self.current_pose.y
        
        # Calcul de la distance euclidienne
        distance = math.sqrt((y_w - y_t)**2 + (x_w - x_t)**2)
        
        return distance
    
    def regulate(self):
        """Régule l'orientation et la distance de la tortue vers le waypoint"""
        if self.current_pose is None:
            return
        
        # Calcul de l'angle désiré
        theta_desired = self.calculate_desired_angle()
        if theta_desired is None:
            return
        
        # Angle actuel de la tortue
        theta = self.current_pose.theta
        
        # Calcul de l'erreur angulaire
        e_angular = math.atan(math.tan((theta_desired - theta) / 2))
        
        # Calcul de l'erreur linéaire (distance)
        e_linear = self.calculate_linear_error()
        if e_linear is None:
            return
            
        # Création du message pour indiquer si la tortue est en mouvement
        is_moving = Bool()
        is_moving.data = e_linear > self.distance_tolerance
        self.is_moving_publisher.publish(is_moving)
        
        # Création de la commande
        cmd = Twist()
        
        # Commande angulaire proportionnelle
        cmd.angular.z = self.Kp * e_angular
        
        # Commande linéaire proportionnelle si la distance est supérieure à la tolérance
        if e_linear > self.distance_tolerance:
            cmd.linear.x = self.Kpl * e_linear
        else:
            # Arrêt si on est suffisamment proche du waypoint
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        # Publication de la commande
        self.cmd_vel_publisher.publish(cmd)
        
        self.get_logger().debug('Regulation: distance={:.2f}, angular_error={:.2f}, linear_command={:.2f}, angular_command={:.2f}'.format(
            e_linear, e_angular, cmd.linear.x, cmd.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = SetWayPoint()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
