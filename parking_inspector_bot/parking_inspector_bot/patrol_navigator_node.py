#!/usr/bin/env python3
"""No ROS2 de navegacao autonoma por Maquina de Estados Finitos (FSM) para inspecao de vagas."""

import math
import time
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, String


def normalize_angle(angle_rad: float) -> float:
    """Normaliza um angulo em radianos para o intervalo [-pi, +pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Converte um quaternion (x, y, z, w) para o angulo de guinada (Euler yaw)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Limita um valor escalar ao intervalo [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def clamp_signed_min(value: float, min_abs: float, max_abs: float) -> float:
    """Aplica saturacao simetrica garantindo modulo minimo para vencer inercia."""
    sign = 1.0 if value >= 0.0 else -1.0
    magnitude = clamp(abs(value), min_abs, max_abs)
    return sign * magnitude


class PatrolNavigatorNode(Node):
    """Controlador FSM de ronda periodica ao longo do corredor do estacionamento."""

    STATE_START_DELAY = 'START_DELAY'
    STATE_MOVING_TO_SPOT = 'MOVING_TO_SPOT'
    STATE_INSPECTING_SPOT = 'INSPECTING_SPOT'
    STATE_RETURNING_TO_BASE = 'RETURNING_TO_BASE'
    STATE_WAIT_NEXT_CYCLE = 'WAIT_NEXT_CYCLE'

    def __init__(self) -> None:
        super().__init__('patrol_navigator_node')

        # Parametros configuraveis da missao e do controlador Proporcional (P)
        self.declare_parameter('waypoints_x', [3.0, 6.0, 9.0, 12.0, 15.0])
        self.declare_parameter('corridor_y', 0.0)
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_yaw', 0.0)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('kp_linear', 0.85)
        self.declare_parameter('kp_angular', 2.00)
        self.declare_parameter('position_tolerance', 0.06)
        self.declare_parameter('yaw_tolerance', 0.035)
        self.declare_parameter('start_delay_sec', 2.0)
        self.declare_parameter('inspect_duration_sec', 2.5)
        self.declare_parameter('patrol_interval_sec', 15.0)
        self.declare_parameter('obstacle_stop_distance_m', 0.40)

        raw_waypoints = self.get_parameter('waypoints_x').value
        self.waypoints_x: List[float] = [float(x) for x in raw_waypoints]
        self.corridor_y: float = float(self.get_parameter('corridor_y').value)
        self.base_x: float = float(self.get_parameter('base_x').value)
        self.base_y: float = float(self.get_parameter('base_y').value)
        self.base_yaw: float = float(self.get_parameter('base_yaw').value)
        self.max_linear_speed: float = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed: float = float(self.get_parameter('max_angular_speed').value)
        self.kp_linear: float = float(self.get_parameter('kp_linear').value)
        self.kp_angular: float = float(self.get_parameter('kp_angular').value)
        self.position_tolerance: float = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance: float = float(self.get_parameter('yaw_tolerance').value)
        self.start_delay_sec: float = float(self.get_parameter('start_delay_sec').value)
        self.inspect_duration_sec: float = float(self.get_parameter('inspect_duration_sec').value)
        self.patrol_interval_sec: float = float(self.get_parameter('patrol_interval_sec').value)
        self.obstacle_stop_distance_m: float = float(
            self.get_parameter('obstacle_stop_distance_m').value
        )

        # Estado estimativo da pose e do sensor frontal
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        self.odom_received: bool = False
        self.front_range_m: float = 5.0
        self.last_obstacle_warn_sec: float = 0.0

        # Variaveis internas da Maquina de Estados Finitos
        self.state: str = self.STATE_START_DELAY
        self.current_spot_index: int = 0
        self.completed_cycles: int = 0
        self.spot_xy_reached: bool = False
        self.base_xy_reached: bool = False
        self.using_ros_clock: bool = False
        self.state_start_sec: float = 0.0
        self.state_timer_initialized: bool = False

        # Assinaturas
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            20,
        )
        self.front_ultrasonic_sub = self.create_subscription(
            Range,
            '/ultrasonic/front',
            self._front_range_callback,
            20,
        )

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.inspection_trigger_pub = self.create_publisher(Int32, '/inspection_trigger', 10)
        self.patrol_state_pub = self.create_publisher(String, '/patrol_state', 10)

        # Loop de controle a 20 Hz
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'No patrol_navigator iniciado. Waypoints X: {self.waypoints_x} m | '
            f'Corredor Y: {self.corridor_y:.2f} m'
        )

    def _now_sec(self) -> float:
        """Retorna o tempo atual em segundos priorizando o clock ROS2/Gazebo."""
        ros_ns = self.get_clock().now().nanoseconds
        if ros_ns > 0:
            now_val = ros_ns * 1e-9
            if not self.using_ros_clock:
                self.using_ros_clock = True
                self.state_start_sec = now_val
            return now_val
        return time.monotonic()

    def _transition_to(self, new_state: str, detail_msg: str = '') -> None:
        """Executa transicao de estado da FSM registrando tempo inicial e log."""
        self.state = new_state
        self.state_start_sec = self._now_sec()
        self._publish_patrol_state()
        if detail_msg:
            self.get_logger().info(f'[FSM -> {new_state}] {detail_msg}')
        else:
            self.get_logger().info(f'[FSM -> {new_state}]')

    def _publish_patrol_state(self) -> None:
        """Publica descricao legivel do estado atual da patrulha em /patrol_state."""
        state_msg = String()
        if self.state in (self.STATE_MOVING_TO_SPOT, self.STATE_INSPECTING_SPOT):
            spot_id = self.current_spot_index + 1
            state_msg.data = f'{self.state} (Vaga {spot_id}/{len(self.waypoints_x)})'
        elif self.state == self.STATE_WAIT_NEXT_CYCLE:
            state_msg.data = f'{self.state} (Ciclo #{self.completed_cycles} concluido)'
        else:
            state_msg.data = self.state
        self.patrol_state_pub.publish(state_msg)

    def _publish_stop(self) -> None:
        """Envia comando de velocidade linear e angular nula em /cmd_vel."""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)

    def _odom_callback(self, msg: Odometry) -> None:
        """Atualiza posicao (X, Y) e orientacao (yaw) a partir de /odom."""
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw = quaternion_to_yaw(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        if not self.odom_received:
            self.odom_received = True
            self.state_start_sec = self._now_sec()
            self.state_timer_initialized = True
            self.get_logger().info(
                f'Odometria recebida em ({self.robot_x:.2f}, {self.robot_y:.2f}). '
                f'Aguardando {self.start_delay_sec:.1f}s na estacao inicial.'
            )

    def _front_range_callback(self, msg: Range) -> None:
        """Atualiza leitura do sensor ultrassonico frontal para anticolisao."""
        raw = float(msg.range)
        if math.isnan(raw):
            return
        max_r = float(msg.max_range) if msg.max_range > 0.0 else 5.0
        if math.isinf(raw) or raw > max_r:
            self.front_range_m = max_r
        elif raw >= max(0.05, float(msg.min_range)):
            self.front_range_m = raw

    def _check_front_obstacle(self, now_sec: float) -> bool:
        """Verifica se ha obstaculo frontal abaixo de 0.40 m e freia imediatamente."""
        if self.front_range_m < self.obstacle_stop_distance_m:
            self._publish_stop()
            if (now_sec - self.last_obstacle_warn_sec) >= 1.0:
                self.last_obstacle_warn_sec = now_sec
                self.get_logger().warn(
                    f'ANTICOLISAO ATIVA: Obstaculo frontal detectado a '
                    f'{self.front_range_m:.2f} m (< {self.obstacle_stop_distance_m:.2f} m). '
                    f'Robo parado aguardando liberacao do corredor!'
                )
            return True
        return False

    def _control_loop(self) -> None:
        """Executa um passo da Maquina de Estados Finitos de navegacao."""
        self._publish_patrol_state()

        if not self.odom_received or not self.state_timer_initialized:
            self._publish_stop()
            return

        now_sec = self._now_sec()
        elapsed_sec = now_sec - self.state_start_sec

        # -----------------------------------------------------------------
        # ESTADO 1: START_DELAY
        # -----------------------------------------------------------------
        if self.state == self.STATE_START_DELAY:
            self._publish_stop()
            if elapsed_sec >= self.start_delay_sec:
                self.current_spot_index = 0
                self.spot_xy_reached = False
                target_x = self.waypoints_x[self.current_spot_index]
                self._transition_to(
                    self.STATE_MOVING_TO_SPOT,
                    f'Iniciando ronda rumo a Vaga 1 em X={target_x:.1f} m.',
                )
            return

        # -----------------------------------------------------------------
        # ESTADO 2: MOVING_TO_SPOT
        # -----------------------------------------------------------------
        if self.state == self.STATE_MOVING_TO_SPOT:
            target_x = self.waypoints_x[self.current_spot_index]
            target_y = self.corridor_y
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            dist_error = math.hypot(dx, dy)

            if not self.spot_xy_reached:
                if dist_error < self.position_tolerance:
                    self.spot_xy_reached = True
                    self._publish_stop()
                else:
                    if self._check_front_obstacle(now_sec):
                        return

                    desired_yaw = math.atan2(dy, dx)
                    yaw_error = normalize_angle(desired_yaw - self.robot_yaw)
                    cmd = Twist()

                    if abs(yaw_error) > 0.35:
                        cmd.linear.x = 0.0
                        cmd.angular.z = clamp_signed_min(
                            self.kp_angular * yaw_error,
                            0.12,
                            self.max_angular_speed,
                        )
                    else:
                        linear_raw = self.kp_linear * dist_error * max(0.25, math.cos(yaw_error))
                        cmd.linear.x = clamp(linear_raw, 0.08, self.max_linear_speed)
                        cmd.angular.z = clamp(
                            self.kp_angular * yaw_error + 0.8 * dy,
                            -self.max_angular_speed,
                            self.max_angular_speed,
                        )
                    self.cmd_vel_pub.publish(cmd)
                    return

            # Alinhamento fino em theta = 0.0 rad antes de disparar o ultrassom lateral
            align_error = normalize_angle(0.0 - self.robot_yaw)
            if abs(align_error) > self.yaw_tolerance:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = clamp_signed_min(
                    self.kp_angular * align_error,
                    0.10,
                    0.65,
                )
                self.cmd_vel_pub.publish(cmd)
                return

            self._publish_stop()
            self.spot_xy_reached = False
            spot_id = self.current_spot_index + 1

            trigger_msg = Int32()
            trigger_msg.data = spot_id
            self.inspection_trigger_pub.publish(trigger_msg)

            self._transition_to(
                self.STATE_INSPECTING_SPOT,
                f'Alinhado na Vaga {spot_id} (X={self.robot_x:.2f} m, Y={self.robot_y:.2f} m). '
                f'Coletando ultrassom por {self.inspect_duration_sec:.1f}s.',
            )
            return

        # -----------------------------------------------------------------
        # ESTADO 3: INSPECTING_SPOT
        # -----------------------------------------------------------------
        if self.state == self.STATE_INSPECTING_SPOT:
            self._publish_stop()
            if elapsed_sec >= self.inspect_duration_sec:
                if (self.current_spot_index + 1) < len(self.waypoints_x):
                    self.current_spot_index += 1
                    self.spot_xy_reached = False
                    next_spot_id = self.current_spot_index + 1
                    next_x = self.waypoints_x[self.current_spot_index]
                    self._transition_to(
                        self.STATE_MOVING_TO_SPOT,
                        f'Avancando para Vaga {next_spot_id} em X={next_x:.1f} m.',
                    )
                else:
                    self.base_xy_reached = False
                    self._transition_to(
                        self.STATE_RETURNING_TO_BASE,
                        f'Inspecao das {len(self.waypoints_x)} vagas concluida! '
                        f'Retornando a estacao inicial ({self.base_x:.1f}, {self.base_y:.1f}).',
                    )
            return

        # -----------------------------------------------------------------
        # ESTADO 4: RETURNING_TO_BASE
        # -----------------------------------------------------------------
        if self.state == self.STATE_RETURNING_TO_BASE:
            dx = self.base_x - self.robot_x
            dy = self.base_y - self.robot_y
            dist_to_base = math.hypot(dx, dy)

            if not self.base_xy_reached:
                if dist_to_base < self.position_tolerance:
                    self.base_xy_reached = True
                    self._publish_stop()
                else:
                    desired_yaw = math.atan2(dy, dx)
                    yaw_error = normalize_angle(desired_yaw - self.robot_yaw)
                    cmd = Twist()

                    if abs(yaw_error) > 0.30:
                        # Gira no proprio eixo antes de deslocar pelo corredor de retorno
                        cmd.linear.x = 0.0
                        cmd.angular.z = clamp_signed_min(
                            self.kp_angular * yaw_error,
                            0.15,
                            self.max_angular_speed,
                        )
                    else:
                        if self._check_front_obstacle(now_sec):
                            return
                        linear_raw = (
                            self.kp_linear * dist_to_base * max(0.25, math.cos(yaw_error))
                        )
                        cmd.linear.x = clamp(linear_raw, 0.08, self.max_linear_speed)
                        cmd.angular.z = clamp(
                            self.kp_angular * yaw_error - 0.8 * dy,
                            -self.max_angular_speed,
                            self.max_angular_speed,
                        )
                    self.cmd_vel_pub.publish(cmd)
                    return

            # Realinha para theta = 0.0 rad na estacao inicial
            final_yaw_error = normalize_angle(self.base_yaw - self.robot_yaw)
            if abs(final_yaw_error) > self.yaw_tolerance:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = clamp_signed_min(
                    self.kp_angular * final_yaw_error,
                    0.12,
                    self.max_angular_speed,
                )
                self.cmd_vel_pub.publish(cmd)
                return

            self._publish_stop()
            self.base_xy_reached = False
            self.completed_cycles += 1
            self._transition_to(
                self.STATE_WAIT_NEXT_CYCLE,
                f'Estacionado na base ({self.robot_x:.2f}, {self.robot_y:.2f}, '
                f'yaw={math.degrees(self.robot_yaw):.1f} deg). '
                f'Proximo ciclo em {self.patrol_interval_sec:.1f}s.',
            )
            return

        # -----------------------------------------------------------------
        # ESTADO 5: WAIT_NEXT_CYCLE
        # -----------------------------------------------------------------
        if self.state == self.STATE_WAIT_NEXT_CYCLE:
            self._publish_stop()
            if elapsed_sec >= self.patrol_interval_sec:
                self.current_spot_index = 0
                self.spot_xy_reached = False
                first_x = self.waypoints_x[self.current_spot_index]
                self._transition_to(
                    self.STATE_MOVING_TO_SPOT,
                    f'Reiniciando ciclo periodico #{self.completed_cycles + 1} '
                    f'rumo a Vaga 1 (X={first_x:.1f} m).',
                )


def main(args=None) -> None:
    """Ponto de entrada do no patrol_navigator."""
    rclpy.init(args=args)
    node = PatrolNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando patrol_navigator_node por solicitacao do usuario.')
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
