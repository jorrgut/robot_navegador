import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')

        # Publicador para comandos de velocidad
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscripciones a odometría y escaneo láser
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'base_scan',
            self.laser_callback,
            10
        )

        # Variables de estado y objetivo
        self.twist = Twist()
        self.current_pose = PoseStamped()
        self.obstacle_detected = False
        self.goal_reached = False

        # Definición del punto objetivo en el marco 'odom'
        self.target_pose_odom = PoseStamped()
        self.target_pose_odom.header.frame_id = 'odom'
        self.target_pose_odom.pose.position.x = 16.75  # Ejemplo: 16 metros hacia adelante desde la posición inicial
        self.target_pose_odom.pose.position.y = 0.0
        self.target_pose_odom.pose.orientation.w = 1.0

        # Parámetros de evasión de obstáculos
        self.min_front_distance = 0.0
        self.rotation_angle = 2.094  # Ángulo de rotación para evitar obstáculo (120 grados)
        self.rotation_time = self.rotation_angle / 0.9  # Tiempo necesario para girar 120 grados a una velocidad angular de 0.9 rad/s
        self.forward_time_after_rotation = 1.0  # Tiempo para avanzar recto después de girar (1 segundo)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.navigate_to_goal()

    def laser_callback(self, msg):
        self.laser_data = msg
        # Detectar obstáculos en el rango frontal
        front_distances = msg.ranges[len(msg.ranges) // 3: 2 * len(msg.ranges) // 3]
        self.min_front_distance = min(front_distances)

        if self.min_front_distance < 0.9:  # Distancia de seguridad para detener y girar aumentada a 0.8 metros
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def navigate_to_goal(self):
        if self.goal_reached:
            return

        # Calcular la distancia y el ángulo hacia el objetivo en odom
        distance_to_goal = math.sqrt((self.target_pose_odom.pose.position.x - self.current_pose.position.x)**2 +
                                     (self.target_pose_odom.pose.position.y - self.current_pose.position.y)**2)

        target_angle = math.atan2(self.target_pose_odom.pose.position.y - self.current_pose.position.y,
                                  self.target_pose_odom.pose.position.x - self.current_pose.position.x)
        current_angle = self.get_yaw_from_pose(self.current_pose)
        angle_diff = self.normalize_angle(target_angle - current_angle)

        # Información de depuración
        self.get_logger().info(f'Current position: ({self.current_pose.position.x}, {self.current_pose.position.y})')
        self.get_logger().info(f'Target position: ({self.target_pose_odom.pose.position.x}, {self.target_pose_odom.pose.position.y})')
        self.get_logger().info(f'Distance to goal: {distance_to_goal}')
        self.get_logger().info(f'Angle to goal: {target_angle}, Current angle: {current_angle}, Angle difference: {angle_diff}')

        if distance_to_goal < 0.1:
            # Detenerse al alcanzar el objetivo
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher_.publish(self.twist)
            self.goal_reached = True
            self.get_logger().info('Goal reached!')
        else:
            # Moverse hacia el objetivo
            self.move_to_goal(angle_diff)

    def move_to_goal(self, angle_diff):
        # Ajustar la dirección y velocidad según la detección de obstáculos
        if self.obstacle_detected:
            self.avoid_obstacle()
        else:
            # Avanzar hacia el objetivo
            self.twist.linear.x = 0.2  # Ajustar velocidad lineal según sea necesario
            self.twist.angular.z = angle_diff * 0.5  # Ajuste proporcional al ángulo
            self.publisher_.publish(self.twist)

    def avoid_obstacle(self):
        # Estrategia de evasión reactiva
        if not hasattr(self, 'escape_start_time'):
            self.escape_start_time = self.get_clock().now()
            self.state = 'rotate_away'

        elapsed_time = self.get_clock().now() - self.escape_start_time

        if self.state == 'rotate_away':
            if elapsed_time.nanoseconds / 1e9 < self.rotation_time:
                # Girar para alejarse del obstáculo
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.9  # Ajustar velocidad angular según sea necesario
            else:
                self.state = 'move_forward'
                self.escape_start_time = self.get_clock().now()

        elif self.state == 'move_forward':
            if elapsed_time.nanoseconds / 1e9 < self.forward_time_after_rotation:
                # Avanzar recto después de girar para alejarse del obstáculo
                self.twist.linear.x = 0.3  # Velocidad lineal hacia adelante
                self.twist.angular.z = 0.0
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                del self.escape_start_time
                self.state = 'none'

        self.publisher_.publish(self.twist)

    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    reactive_navigator = ReactiveNavigator()
    rclpy.spin(reactive_navigator)
    reactive_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

