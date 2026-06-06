import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


def novel_downscaling(data: np.ndarray, k: int = 20, threshold: float = 0.3) -> np.ndarray:
    n = len(data)
    diff_list = []
    i = 0
    while i < n - 2:
        j = i + 1
        diff = data[i] - data[j]
        if abs(diff) > threshold:
            k_idx = j + 1
            diff_2 = data[j] - data[k_idx] if k_idx < n else 0.0
            while k_idx < n - 1 and abs(diff_2) > threshold and diff_2 * diff > 0:
                k_idx += 1
                j += 1
                diff_2 = data[j] - data[k_idx] if k_idx < n else 0.0
            diff_list.append((j, abs(data[j] - data[i])))
            diff_list.append((i, abs(data[j] - data[i])))
            i = j
        else:
            i += 1
    diff_list.sort(key=lambda x: x[1], reverse=True)
    mid = n // 2
    critical_points = [
        int(np.argmin(data[:mid])),
        int(np.argmin(data[mid:])) + mid,
    ]
    for idx, _ in diff_list:
        if len(critical_points) >= k:
            break
        if idx not in critical_points and 0 <= idx < n:
            critical_points.append(idx)
    for idx in range(0, n, max(1, n // k)):
        if len(critical_points) >= k:
            break
        if idx not in critical_points:
            critical_points.append(idx)
    critical_points = sorted(set(critical_points))[:k]
    return data[critical_points]


class F1TenthNavigator(Node):
    """
    Wall following con 3 estados:
      FOLLOW  → seguir pared derecha normal
      RECOVER → girar izquierda al detectar curva
      REVERSE → reversa + giro para salir de esquina
    """

    STATE_FOLLOW  = 'following'
    STATE_RECOVER = 'recovering'
    STATE_REVERSE = 'reversing'

    def __init__(self):
        super().__init__('f1tenth_navigator')

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # ── Índices LiDAR (1080 pts, FOV 270°) ───────────────────────────────
        self.IDX_FRONT    = 540
        self.IDX_RIGHT_90 = 270
        self.IDX_RIGHT_45 = 405
        self.IDX_LEFT_90  = 810

        # ── Parámetros wall following ─────────────────────────────────────────
        self.target_distance   = 0.8
        self.kp                = 1.0
        self.kd                = 0.05
        self.max_speed         = 1.5
        self.min_speed         = 0.5
        self.k                 = 20
        self.threshold         = 0.3

        # ── Umbrales de detección ─────────────────────────────────────────────
        self.left_curve_dist   = 1.5
        self.front_clear_dist  = 1.5

        # ── Estado ───────────────────────────────────────────────────────────
        self.state             = self.STATE_FOLLOW
        self.recover_steps     = 0
        self.recover_steps_max = 30

        self.reverse_steps     = 0
        self.reverse_steps_max = 35   # duración total de la maniobra

        # Detector de atasco: frente no mejora → reversa
        # UMBRAL BAJO (8) para reaccionar rápido como en la imagen
        self.stuck_steps       = 0
        self.stuck_steps_max   = 8
        self.prev_front_dist   = 99.0

        self.prev_error = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    def callback(self, msg):
        raw = np.array(msg.ranges, dtype=np.float32)
        raw = np.nan_to_num(raw, nan=msg.range_max, posinf=msg.range_max)
        raw = np.clip(raw, msg.range_min, msg.range_max)

        obs = novel_downscaling(raw, k=self.k, threshold=self.threshold)

        dist_front    = float(raw[self.IDX_FRONT])
        dist_right_90 = float(raw[self.IDX_RIGHT_90])
        dist_right_45 = float(raw[self.IDX_RIGHT_45])
        dist_left_90  = float(raw[self.IDX_LEFT_90])
        min_dist      = float(np.min(obs))

        # ── Detector de atasco (activo en FOLLOW y RECOVER) ───────────────────
        if self.state != self.STATE_REVERSE:
            if dist_front < 0.5 and abs(dist_front - self.prev_front_dist) < 0.05:
                self.stuck_steps += 1
            else:
                self.stuck_steps = 0
            self.prev_front_dist = dist_front

            if self.stuck_steps >= self.stuck_steps_max:
                self.state         = self.STATE_REVERSE
                self.reverse_steps = 0
                self.stuck_steps   = 0
                self.get_logger().warn(
                    f"ATASCADO — iniciando reversa  front={dist_front:.2f}")

        # ── Transición FOLLOW → RECOVER ───────────────────────────────────────
        if self.state == self.STATE_FOLLOW:
            left_curve = (
                dist_left_90 < self.left_curve_dist and
                dist_front   < self.front_clear_dist
            )
            if left_curve:
                self.state         = self.STATE_RECOVER
                self.recover_steps = 0
                self.get_logger().warn(
                    f"Curva izq — L={dist_left_90:.2f}  front={dist_front:.2f}")

        # ── Ejecutar estado ───────────────────────────────────────────────────
        if self.state == self.STATE_REVERSE:
            speed, steering = self._reverse(dist_front, min_dist)
        elif self.state == self.STATE_RECOVER:
            speed, steering = self._recover(dist_front, dist_left_90)
        else:
            speed, steering = self._wall_follow(dist_right_90, dist_right_45, dist_front)

        self.publish_drive(speed, steering)
        self.get_logger().info(
            f"[{self.state}]  front={dist_front:.2f}  "
            f"R={dist_right_90:.2f}  L={dist_left_90:.2f}  "
            f"steer={steering:.3f}  speed={speed:.2f}  "
            f"stuck={self.stuck_steps}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _wall_follow(self, dist_right_90, dist_right_45, dist_front):
        alpha = np.arctan2(
            dist_right_45 * np.cos(np.radians(45)) - dist_right_90,
            dist_right_45 * np.sin(np.radians(45))
        )
        dist_wall_now  = dist_right_90 * np.cos(alpha)
        dist_wall_next = dist_wall_now + 1.0 * np.sin(alpha)

        error           = self.target_distance - dist_wall_next
        d_error         = error - self.prev_error
        self.prev_error = error

        steering = float(np.clip(self.kp * error + self.kd * d_error, -0.4, 0.4))

        speed = self.max_speed
        if dist_front < 1.5:
            speed = self.min_speed + (self.max_speed - self.min_speed) * (dist_front / 1.5)
        speed = float(np.clip(speed - 0.5 * abs(steering), self.min_speed, self.max_speed))

        return speed, steering

    # ─────────────────────────────────────────────────────────────────────────
    def _recover(self, dist_front, dist_left_90):
        self.recover_steps += 1
        steering = -0.4
        speed    = 0.4

        recovered = (
            dist_front   > self.front_clear_dist * 1.3 and
            dist_left_90 > self.left_curve_dist  * 0.9
        )
        if recovered or self.recover_steps > self.recover_steps_max:
            self.state      = self.STATE_FOLLOW
            self.prev_error = 0.0
            self.get_logger().info("Recuperación completada")

        return speed, steering

    # ─────────────────────────────────────────────────────────────────────────
    def _reverse(self, dist_front, min_dist):
        """
        Maniobra de 3 fases para salir de una esquina en L:
          Fase 1 → reversa recta           (aleja de la pared frontal)
          Fase 2 → reversa girando derecha (abre ángulo)
          Fase 3 → avanza girando izquierda (retoma dirección)
        """
        self.reverse_steps += 1
        t = self.reverse_steps

        phase1 = self.reverse_steps_max // 3
        phase2 = self.reverse_steps_max * 2 // 3

        if t <= phase1:
            # Fase 1: reversa recta
            speed    = -0.5
            steering =  0.0
        elif t <= phase2:
            # Fase 2: reversa girando derecha
            speed    = -0.5
            steering =  0.4
        else:
            # Fase 3: avanzar girando izquierda
            speed    =  0.5
            steering = -0.4

        # Salir cuando el frente esté despejado después de la fase 3
        if t > phase2 and dist_front > self.front_clear_dist:
            self.state      = self.STATE_FOLLOW
            self.prev_error = 0.0
            self.stuck_steps = 0
            self.get_logger().info("Reversa completada — retomando wall following")

        return speed, steering

    # ─────────────────────────────────────────────────────────────────────────
    def publish_drive(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = 'base_link'
        msg.drive.speed          = speed
        msg.drive.steering_angle = steering
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = F1TenthNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

