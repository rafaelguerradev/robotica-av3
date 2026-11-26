#!/usr/bin/env python3
"""No ROS2 de ponte serial (Hardware Bridge) entre o ESP32-WROOM-32, RViz2 e Sistema Central."""

from datetime import datetime
import json
import math
import time
from typing import Optional

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
import serial
from serial import SerialException
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion_zw(yaw_rad: float) -> tuple[float, float]:
    """Converte angulo yaw (rad) nas componentes (qz, qw) de um quaternion planar."""
    half = 0.5 * yaw_rad
    return math.sin(half), math.cos(half)


class HardwareBridgeNode(Node):
    """Conecta o firmware serial do ESP32-WROOM aos topicos padrao do pacote ROS2."""

    def __init__(self) -> None:
        super().__init__('hardware_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('right_sensor_frame', 'ultrasonic_right_link')
        self.declare_parameter('front_sensor_frame', 'ultrasonic_front_link')
        self.declare_parameter('ultrasonic_min_range_m', 0.02)
        self.declare_parameter('ultrasonic_max_range_m', 3.50)
        self.declare_parameter('ultrasonic_fov_rad', 0.26)

        self.serial_port_name: str = str(self.get_parameter('serial_port').value)
        self.baud_rate: int = int(self.get_parameter('baud_rate').value)
        self.reconnect_interval_sec: float = float(
            self.get_parameter('reconnect_interval_sec').value
        )
        self.odom_frame: str = str(self.get_parameter('odom_frame').value)
        self.base_frame: str = str(self.get_parameter('base_frame').value)
        self.right_sensor_frame: str = str(self.get_parameter('right_sensor_frame').value)
        self.front_sensor_frame: str = str(self.get_parameter('front_sensor_frame').value)
        self.min_range_m: float = float(self.get_parameter('ultrasonic_min_range_m').value)
        self.max_range_m: float = float(self.get_parameter('ultrasonic_max_range_m').value)
        self.fov_rad: float = float(self.get_parameter('ultrasonic_fov_rad').value)

        self.serial_conn: Optional[serial.Serial] = None
        self.last_reconnect_attempt_sec: float = 0.0
        self.last_port_warn_sec: float = 0.0

        # Estado cinematica espelhado do prototipo fisico
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        self.linear_vx: float = 0.0
        self.angular_wz: float = 0.0
        self.wheel_angle_rad: float = 0.0
        self.last_tf_pub_sec: float = time.monotonic()

        # Publicadores ROS2 compativeis com central_monitor_node e RViz2
        self.right_range_pub = self.create_publisher(Range, '/ultrasonic/right', 20)
        self.front_range_pub = self.create_publisher(Range, '/ultrasonic/front', 20)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.spot_update_pub = self.create_publisher(String, '/spot_status_update', 10)
        self.patrol_state_pub = self.create_publisher(String, '/patrol_state', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Assinaturas para enviar comandos do ROS2 ao ESP32-WROOM
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
        )
        self.raw_cmd_sub = self.create_subscription(
            String,
            '/esp32_command',
            self._esp32_raw_command_callback,
            10,
        )

        # Timers de leitura serial (50 Hz) e manutencao da arvore TF/JointStates (20 Hz)
        self.serial_poll_timer = self.create_timer(0.02, self._poll_serial_port)
        self.tf_keepalive_timer = self.create_timer(0.05, self._publish_kinematic_state)

        self._try_connect_serial()
        self.get_logger().info(
            f'No hardware_bridge iniciado | Porta alvo: {self.serial_port_name} @ '
            f'{self.baud_rate} baud'
        )

    def _try_connect_serial(self) -> None:
        """Abre ou reconecta de forma segura a porta serial USB do ESP32-WROOM."""
        now_sec = time.monotonic()
        if (now_sec - self.last_reconnect_attempt_sec) < self.reconnect_interval_sec:
            return
        self.last_reconnect_attempt_sec = now_sec

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=0.01,
                write_timeout=0.05,
            )
            self.get_logger().info(
                f'Conectado com sucesso ao ESP32-WROOM em {self.serial_port_name} '
                f'({self.baud_rate} baud)!'
            )
            state_msg = String()
            state_msg.data = f'ESP32_CONECTADO ({self.serial_port_name})'
            self.patrol_state_pub.publish(state_msg)
        except (SerialException, OSError) as exc:
            self.serial_conn = None
            if (now_sec - self.last_port_warn_sec) >= 5.0:
                self.last_port_warn_sec = now_sec
                self.get_logger().warn(
                    f'Aguardando conexao do ESP32 em {self.serial_port_name}: {exc}'
                )

    def _write_serial_line(self, line_text: str) -> None:
        """Envia uma linha terminada em \\n para o ESP32 caso a porta esteja aberta."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        payload_bytes = (line_text.strip() + '\n').encode('utf-8')
        try:
            self.serial_conn.write(payload_bytes)
        except (SerialException, OSError) as exc:
            self.get_logger().error(f'Erro de escrita na Serial {self.serial_port_name}: {exc}')
            self._close_serial()

    def _close_serial(self) -> None:
        """Fecha a instancia serial atual em caso de desconexao fisica."""
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except (SerialException, OSError) as exc:
                self.get_logger().debug(f'Aviso ao fechar porta serial: {exc}')
            self.serial_conn = None

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Converte /cmd_vel do ROS2 em JSON Modo B para o ESP32-WROOM."""
        cmd_json = json.dumps(
            {
                'vx': round(float(msg.linear.x), 3),
                'wz': round(float(msg.angular.z), 3),
            }
        )
        self._write_serial_line(cmd_json)

    def _esp32_raw_command_callback(self, msg: String) -> None:
        """Permite enviar START, STOP, SET_LARGURA:1.60 ou SET_COMPRIMENTO:0.68 via ROS2."""
        command_str = msg.data.strip()
        if command_str:
            self.get_logger().info(f'Enviando comando ao ESP32: {command_str}')
            self._write_serial_line(command_str)

    def _build_range_msg(self, frame_id: str, distance_m: float) -> Range:
        """Constroi mensagem sensor_msgs/Range para visualizacao no RViz2."""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov_rad
        msg.min_range = self.min_range_m
        msg.max_range = self.max_range_m
        msg.range = max(self.min_range_m, min(self.max_range_m, float(distance_m)))
        return msg

    def _publish_kinematic_state(self) -> None:
        """Publica /odom, TF (odom -> base_footprint) e /joint_states para o RViz2."""
        now_mono = time.monotonic()
        dt = max(0.0, min(0.2, now_mono - self.last_tf_pub_sec))
        self.last_tf_pub_sec = now_mono

        if abs(self.linear_vx) > 0.005:
            self.wheel_angle_rad = (self.wheel_angle_rad + (self.linear_vx / 0.09) * dt) % (
                2.0 * math.pi
            )

        stamp = self.get_clock().now().to_msg()
        qz, qw = yaw_to_quaternion_zw(self.robot_yaw)

        # 1. Transformacao TF odom -> base_footprint
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.robot_x
        tf_msg.transform.translation.y = self.robot_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # 2. Estados das juntas continuas das rodas para o robot_state_publisher
        js_msg = JointState()
        js_msg.header.stamp = stamp
        js_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        js_msg.position = [self.wheel_angle_rad, self.wheel_angle_rad]
        self.joint_state_pub.publish(js_msg)

        # 3. Mensagem nav_msgs/Odometry em /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.linear_vx
        odom_msg.twist.twist.angular.z = self.angular_wz
        self.odom_pub.publish(odom_msg)

    def _poll_serial_port(self) -> None:
        """Le pacotes JSON enviados pelo ESP32-WROOM sem bloquear o executor ROS2."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._try_connect_serial()
            return

        try:
            lines_processed = 0
            while self.serial_conn.in_waiting > 0 and lines_processed < 15:
                raw_bytes = self.serial_conn.readline()
                lines_processed += 1
                if not raw_bytes:
                    break
                decoded_line = raw_bytes.decode('utf-8', errors='replace').strip()
                if decoded_line:
                    self._handle_serial_line(decoded_line)
        except (SerialException, OSError) as exc:
            self.get_logger().warn(
                f'Conexao serial interrompida ({self.serial_port_name}): {exc}'
            )
            self._close_serial()

    def _handle_serial_line(self, line: str) -> None:
        """Interpreta JSON estruturado ou linhas de relatorio vindas do ESP32."""
        if not line.startswith('{'):
            self.get_logger().info(f'[ESP32] {line}')
            return

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().debug(f'Linha JSON parcial ignorada: {line}')
            return

        event_type = str(data.get('event', ''))

        if event_type == 'TELEMETRY':
            self.robot_x = float(data.get('odom_x', self.robot_x))
            self.robot_yaw = float(data.get('odom_yaw', self.robot_yaw))
            self.linear_vx = float(data.get('vx', 0.0))
            self.angular_wz = float(data.get('wz', 0.0))

            dist_right = float(data.get('dist_right', self.max_range_m))
            dist_front = float(data.get('dist_front', self.max_range_m))

            self.right_range_pub.publish(
                self._build_range_msg(self.right_sensor_frame, dist_right)
            )
            self.front_range_pub.publish(
                self._build_range_msg(self.front_sensor_frame, dist_front)
            )

            state_label = str(data.get('state', 'ESP32_ATIVO'))
            patrol_msg = String()
            patrol_msg.data = f'PROTOTIPO_FISICO: {state_label}'
            self.patrol_state_pub.publish(patrol_msg)
            return

        if event_type == 'SPOT_INSPECTED':
            spot_id = int(data.get('spot_id', 1))
            mean_dist = float(data.get('mean_dist_m', 0.0))
            std_dev = float(data.get('std_dev_m', 0.0))
            samples = int(data.get('samples', 15))
            status = str(data.get('status', 'LIVRE'))
            vaga_largura = float(data.get('vaga_largura_m', 1.60))
            vaga_comprimento = float(data.get('vaga_comprimento_m', 0.68))

            self.right_range_pub.publish(
                self._build_range_msg(self.right_sensor_frame, mean_dist)
            )

            bridge_payload = {
                'spot_id': spot_id,
                'status': status,
                'mean_distance_m': round(mean_dist, 3),
                'std_dev_m': round(std_dev, 3),
                'samples': samples,
                'vaga_largura_m': round(vaga_largura, 2),
                'vaga_comprimento_m': round(vaga_comprimento, 2),
                'timestamp': datetime.now().isoformat(timespec='seconds'),
            }

            update_msg = String()
            update_msg.data = json.dumps(bridge_payload, ensure_ascii=False)
            self.spot_update_pub.publish(update_msg)

            self.get_logger().info(
                f'[ESP32 -> ROS2] Vaga {spot_id}: {status} '
                f'(media={mean_dist:.2f} m, largura={vaga_largura:.2f} m, '
                f'comp={vaga_comprimento:.2f} m)'
            )
            return

        if event_type in ('MISSION_STARTED', 'CYCLE_COMPLETED', 'CONFIG_UPDATED', 'BOOT_READY'):
            self.get_logger().info(f'[ESP32 {event_type}] {line}')
            patrol_msg = String()
            patrol_msg.data = f'PROTOTIPO_FISICO: {event_type}'
            self.patrol_state_pub.publish(patrol_msg)
            return

        if event_type in ('EMERGENCY_STOP', 'OBSTACLE_ALERT'):
            self.get_logger().warn(f'[ESP32 ALERTA {event_type}] {line}')
            patrol_msg = String()
            patrol_msg.data = f'ALERTA_FISICO: {event_type}'
            self.patrol_state_pub.publish(patrol_msg)

    def destroy_node(self) -> None:
        """Garante fechamento limpo da porta serial ao encerrar o no."""
        self._close_serial()
        super().destroy_node()


def main(args=None) -> None:
    """Ponto de entrada do no hardware_bridge."""
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando hardware_bridge_node por solicitacao do usuario.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
