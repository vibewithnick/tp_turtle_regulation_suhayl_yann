#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import SetWayPoint
from std_msgs.msg import Bool

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        
        # Client pour le service
        self.client = self.create_client(SetWayPoint, 'set_waypoint_service')
        
        # Attendre que le service soit disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service set_waypoint_service not available, waiting...')
        
        # Subscriber pour savoir si la tortue est en mouvement
        self.is_moving_subscription = self.create_subscription(
            Bool,
            'is_moving',
            self.is_moving_callback,
            10)
            
        self.is_moving = False
        
    def is_moving_callback(self, msg):
        """Callback pour le topic is_moving"""
        self.is_moving = msg.data
    
    def send_request(self, x, y):
        """Envoie une requête au service"""
        request = SetWayPoint.Request()
        request.x = float(x)
        request.y = float(y)
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = WaypointClient()
    
    try:
        # Vérifier les arguments
        if len(sys.argv) != 3:
            client.get_logger().error('Usage: ros2 run turtle_regulation waypoint_client X Y')
            return 1
            
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        
        # Envoyer la requête
        client.get_logger().info('Sending waypoint request: ({}, {})'.format(x, y))
        
        # Attendre que la tortue ne soit pas en mouvement
        while client.is_moving:
            client.get_logger().info('Waiting for turtle to stop moving...')
            rclpy.spin_once(client, timeout_sec=1.0)
        
        response = client.send_request(x, y)
        
        client.get_logger().info('Response: {}'.format('Success' if response.res else 'Failed'))
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
