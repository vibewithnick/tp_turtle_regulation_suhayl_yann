#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
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
        
        # Constante Kp pour la régulation proportionnelle
        self.Kp = 1.0
        
        # Attributs pour stocker la pose courante
        self.current_pose = None
        
        self.get_logger().info('Set waypoint node has been initialized with waypoint: ({}, {})'.format(
            self.waypoint[0], self.waypoint[1]))

    def pose_callback(self, msg):
        """Callback pour le topic pose"""
        self.current_pose = msg
        self.regulate_heading()
    
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
    
    def regulate_heading(self):
        """Régule l'orientation de la tortue vers le waypoint"""
        if self.current_pose is None:
            return
        
        # Calcul de l'angle désiré
        theta_desired = self.calculate_desired_angle()
        if theta_desired is None:
            return
        
        # Angle actuel de la tortue
        theta = self.current_pose.theta
        
        # Calcul de l'erreur
        e = math.atan(math.tan((theta_desired - theta) / 2))
        
        # Commande proportionnelle
        u = self.Kp * e
        
        # Création et publication de la commande
        cmd = Twist()
        cmd.angular.z = u
        self.cmd_vel_publisher.publish(cmd)
        
        self.get_logger().debug('Heading regulation: desired={:.2f}, current={:.2f}, error={:.2f}, command={:.2f}'.format(
            theta_desired, theta, e, u))

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
