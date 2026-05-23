import rclpy
import numpy as np

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class Controller(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.max_lidar_dist = 3000000
        self.StatusLidar = None
        self.preprocess_conv_size=3
        
        self.laser_sub = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,'/drive',10)
        
    def proprocess_lidar(self,ranges):
        self.radians_per_element = (2*np.pi)/len(ranges)
        proc_ranges = np.array(ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.preprocess_conv_size), 'same') / self.preprocess_conv_size
        proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_dist)
        return proc_ranges
        
    def laser_callback(self,scan_msg:LaserScan):
        #1. Convertir los datos a algo facil de usar
        lista_rayos = list(scan_msg.ranges)
        self.get_logger().info(f"Total de rayos recibidos: {len(lista_rayos)}")
        
        rebanada_frente = lista_rayos[500:580]
        distancia_frente = sum(rebanada_frente)/len(rebanada_frente) #por que tenemos que calcular el promedio?
        
        rebanada_izquierda = lista_rayos[900:1000]
        distancia_izquierda = sum(rebanada_izquierda)/len(rebanada_izquierda)
        
        rebanada_derecha = lista_rayos[80:180]
        distancia_derecha = sum(rebanada_derecha)/len(rebanada_derecha)
        
        velocidad = 0.0
        angulo = 0.0
        
        self.get_logger().info(f"Frente: {distancia_frente:.2f} | Izq: {distancia_izquierda:.2f} | Der: {distancia_derecha:.2f}")
        
        if distancia_frente > 1.5:
            velocidad = 1.0
            angulo = 0.0
            self.get_logger().info(f"Frente: {distancia_frente:.2f} | Izq: {distancia_izquierda:.2f} | Der: {distancia_derecha:.2f}")
        else:
            velocidad = 0.4
            self.get_logger().info(f"Frente: {distancia_frente:.2f} | Izq: {distancia_izquierda:.2f} | Der: {distancia_derecha:.2f}")
            if distancia_izquierda > distancia_derecha:
                angulo = 0.4
                self.get_logger().info(f"Frente: {distancia_frente:.2f} | Izq: {distancia_izquierda:.2f} | Der: {distancia_derecha:.2f}")
            else:
                angulo = -0.4
                self.get_logger().info(f"Frente: {distancia_frente:.2f} | Izq: {distancia_izquierda:.2f} | Der: {distancia_derecha:.2f}")
                
                
        msg = AckermannDriveStamped()
        msg.drive.speed = velocidad
        msg.drive.steering_angle = angulo
        self.drive_pub.publish(msg)
    
          
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        
        
        
if __name__ == '__main__':
    main()        
        
        
        
       
        
