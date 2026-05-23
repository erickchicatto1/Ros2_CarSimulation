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
        promedio = sum(lista_rayos)/len(lista_rayos)
        #2. Extraer la distancia de las zonas clave (frente,izquierda,derecha)
        #3. Tomar una decision basada en la distancia
        #4. Crear el mensaje ackermann y publicarlo
        if lista_rayos == lista_rayos[500:580]:
            self.StatusLidar = 'frente'
            
        elif lista_rayos == lista_rayos[900:1000]:
            self.StatusLidar = 'izquierda'
        elif lista_rayos == lista_rayos[80:180]:
            self.StatusLidar = 'derecha'
        else:
            print('Otro valor')
        
                
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
        
        
        
       
        
