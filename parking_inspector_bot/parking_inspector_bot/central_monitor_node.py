#!/usr/bin/env python3
"""Sistema Central de Monitoramento de Estacionamento (Painel CLI + JSON + RViz2 3D)."""

from datetime import datetime
import json
import math
import os
from typing import Any, Dict, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Converte quaternion de orientacao para angulo yaw em graus."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class CentralMonitorNode(Node):
    """Consolida o status das 5 vagas em tempo real no terminal, arquivo JSON e RViz2."""

    def __init__(self) -> None:
        super().__init__('central_monitor_node')

        self.declare_parameter('spots_x', [3.0, 6.0, 9.0, 12.0, 15.0])
        self.declare_parameter('spots_center_y', -3.5)
        self.declare_parameter('json_output_path', '/tmp/parking_central_status.json')

        raw_spots_x = self.get_parameter('spots_x').value
        self.spots_x: List[float] = [float(x) for x in raw_spots_x]
        self.spots_center_y: float = float(self.get_parameter('spots_center_y').value)
        self.json_output_path: str = str(self.get_parameter('json_output_path').value)

        # Estado consolidado das 5 vagas
        self.spots_data: Dict[int, Dict[str, Any]] = {}
        for idx, spot_x in enumerate(self.spots_x, start=1):
            self.spots_data[idx] = {
                'spot_id': idx,
                'center_x': spot_x,
                'center_y': self.spots_center_y,
                'status': 'NAO_INSPECIONADA',
                'mean_distance_m': None,
                'std_dev_m': None,
                'samples': 0,
                'timestamp': '-',
            }

        # Telemetria atual do robo
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw_deg: float = 0.0
        self.patrol_state: str = 'INICIALIZANDO'
        self.last_patrol_state_rendered: str = ''

        # Assinaturas
        self.spot_update_sub = self.create_subscription(
            String,
            '/spot_status_update',
            self._spot_status_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            20,
        )
        self.patrol_state_sub = self.create_subscription(
            String,
            '/patrol_state',
            self._patrol_state_callback,
            10,
        )

        # Publicador de marcadores 3D para o RViz2
        self.markers_pub = self.create_publisher(MarkerArray, '/parking_markers', 10)

        # Timers para atualizacao continua de marcadores RViz2 (2 Hz) e painel (4 s)
        self.markers_timer = self.create_timer(0.5, self._publish_rviz_markers)
        self.dashboard_timer = self.create_timer(4.0, self._periodic_dashboard_sync)

        self._save_status_json()
        self._render_terminal_dashboard()
        self.get_logger().info(
            f'Sistema Central de Monitoramento ativo. Arquivo de status: {self.json_output_path}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Atualiza pose instantanea do robo para exibicao no painel central."""
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.robot_yaw_deg = quaternion_to_yaw_deg(
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )

    def _patrol_state_callback(self, msg: String) -> None:
        """Atualiza estado atual da ronda autonoma."""
        self.patrol_state = msg.data
        if self.patrol_state != self.last_patrol_state_rendered:
            self.last_patrol_state_rendered = self.patrol_state
            self._save_status_json()

    def _spot_status_callback(self, msg: String) -> None:
        """Processa atualizacao JSON de uma vaga vinda do ultrasonic_detector_node."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Falha ao decodificar JSON em /spot_status_update: {exc}')
            return

        spot_id = int(payload.get('spot_id', 0))
        if spot_id not in self.spots_data:
            self.get_logger().warn(f'ID de vaga desconhecido recebido: {spot_id}')
            return

        self.spots_data[spot_id].update(
            {
                'status': str(payload.get('status', 'NAO_INSPECIONADA')),
                'mean_distance_m': payload.get('mean_distance_m'),
                'std_dev_m': payload.get('std_dev_m'),
                'samples': int(payload.get('samples', 0)),
                'timestamp': str(payload.get('timestamp', '-')),
            }
        )

        self._save_status_json()
        self._publish_rviz_markers()
        self._render_terminal_dashboard()

    def _periodic_dashboard_sync(self) -> None:
        """Persiste JSON periodicamente e mantem painel atualizado no terminal."""
        self._save_status_json()
        self._render_terminal_dashboard()

    def _save_status_json(self) -> None:
        """Salva o estado completo do estacionamento em /tmp/parking_central_status.json."""
        free_count = sum(1 for s in self.spots_data.values() if s['status'] == 'LIVRE')
        occupied_count = sum(1 for s in self.spots_data.values() if s['status'] == 'OCUPADA')
        uninspected_count = len(self.spots_data) - free_count - occupied_count

        report = {
            'updated_at': datetime.now().isoformat(timespec='seconds'),
            'patrol_state': self.patrol_state,
            'robot_pose': {
                'x_m': round(self.robot_x, 3),
                'y_m': round(self.robot_y, 3),
                'yaw_deg': round(self.robot_yaw_deg, 2),
            },
            'summary': {
                'total_spots': len(self.spots_data),
                'free_spots': free_count,
                'occupied_spots': occupied_count,
                'uninspected_spots': uninspected_count,
            },
            'spots': list(self.spots_data.values()),
        }

        temp_path = f'{self.json_output_path}.tmp'
        try:
            with open(temp_path, 'w', encoding='utf-8') as file_obj:
                json.dump(report, file_obj, indent=2, ensure_ascii=False)
            os.replace(temp_path, self.json_output_path)
        except OSError as exc:
            self.get_logger().error(f'Erro ao gravar {self.json_output_path}: {exc}')

    def _render_terminal_dashboard(self) -> None:
        """Exibe tabela formatada em tempo real no terminal com o resumo das 5 vagas."""
        free_count = sum(1 for s in self.spots_data.values() if s['status'] == 'LIVRE')
        occupied_count = sum(1 for s in self.spots_data.values() if s['status'] == 'OCUPADA')
        pending_count = len(self.spots_data) - free_count - occupied_count

        sep = '=' * 82
        thin = '-' * 82
        lines = [
            '',
            sep,
            '      SISTEMA CENTRAL DE MONITORAMENTO DE ESTACIONAMENTO AUTONOMO (ROS2)',
            sep,
            f' Estado da Missao : {self.patrol_state}',
            (
                f' Pose do Robo     : X = {self.robot_x:6.2f} m | '
                f'Y = {self.robot_y:6.2f} m | Yaw = {self.robot_yaw_deg:6.1f} deg'
            ),
            (
                f' Resumo de Vagas  : LIVRES = {free_count} | '
                f'OCUPADAS = {occupied_count} | PENDENTES = {pending_count} '
                f'(Total: {len(self.spots_data)})'
            ),
            thin,
            f' {"VAGA":<8} | {"POSICAO (X,Y)":<15} | {"STATUS":<17} | {"MEDIA (m)":<10} | {"AMOSTRAS":<8} | {"HORARIO":<8}',
            thin,
        ]

        for spot_id in sorted(self.spots_data.keys()):
            info = self.spots_data[spot_id]
            pos_str = f'({info["center_x"]:4.1f}, {info["center_y"]:4.1f})'
            status_label = info['status']
            if info['mean_distance_m'] is not None:
                mean_str = f'{info["mean_distance_m"]:6.3f} m'
            else:
                mean_str = '   --   '
            ts_short = info['timestamp'][-8:] if len(info['timestamp']) >= 8 else info['timestamp']
            lines.append(
                f' Vaga {spot_id:<3} | {pos_str:<15} | {status_label:<17} | '
                f'{mean_str:<10} | {info["samples"]:^8} | {ts_short:<8}'
            )

        lines.append(sep)
        print('\n'.join(lines), flush=True)

    def _publish_rviz_markers(self) -> None:
        """Publica MarkerArray em /parking_markers com cubos 2.8x4.8x0.08m e textos 3D."""
        now_stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        # Marcador visual da Estacao de Inicio e Retorno em (0.0, 0.0)
        base_marker = Marker()
        base_marker.header.frame_id = 'odom'
        base_marker.header.stamp = now_stamp
        base_marker.ns = 'base_station'
        base_marker.id = 0
        base_marker.type = Marker.CUBE
        base_marker.action = Marker.ADD
        base_marker.pose.position.x = 0.0
        base_marker.pose.position.y = 0.0
        base_marker.pose.position.z = 0.02
        base_marker.pose.orientation.w = 1.0
        base_marker.scale.x = 1.20
        base_marker.scale.y = 1.20
        base_marker.scale.z = 0.04
        base_marker.color.r = 0.15
        base_marker.color.g = 0.65
        base_marker.color.b = 0.95
        base_marker.color.a = 0.65
        marker_array.markers.append(base_marker)

        base_text = Marker()
        base_text.header.frame_id = 'odom'
        base_text.header.stamp = now_stamp
        base_text.ns = 'base_station_label'
        base_text.id = 1
        base_text.type = Marker.TEXT_VIEW_FACING
        base_text.action = Marker.ADD
        base_text.pose.position.x = 0.0
        base_text.pose.position.y = 0.0
        base_text.pose.position.z = 1.20
        base_text.pose.orientation.w = 1.0
        base_text.scale.z = 0.28
        base_text.color.r = 0.85
        base_text.color.g = 0.95
        base_text.color.b = 1.00
        base_text.color.a = 1.00
        base_text.text = 'ESTACAO BASE\n(0.0, 0.0)'
        marker_array.markers.append(base_text)

        # Cubos 2.8 x 4.8 x 0.08 m e textos flutuantes (Z = 1.8 m) para cada uma das 5 vagas
        for spot_id, info in self.spots_data.items():
            spot_x = float(info['center_x'])
            spot_y = float(info['center_y'])
            status = str(info['status'])

            cube = Marker()
            cube.header.frame_id = 'odom'
            cube.header.stamp = now_stamp
            cube.ns = 'parking_spots'
            cube.id = spot_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = spot_x
            cube.pose.position.y = spot_y
            cube.pose.position.z = 0.04
            cube.pose.orientation.x = 0.0
            cube.pose.orientation.y = 0.0
            cube.pose.orientation.z = 0.0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 2.80
            cube.scale.y = 4.80
            cube.scale.z = 0.08

            if status == 'LIVRE':
                cube.color.r = 0.12
                cube.color.g = 0.82
                cube.color.b = 0.28
                cube.color.a = 0.78
                dist_val = info['mean_distance_m']
                label_text = f'VAGA {spot_id}\nLIVRE ({dist_val:.2f}m)'
            elif status == 'OCUPADA':
                cube.color.r = 0.90
                cube.color.g = 0.16
                cube.color.b = 0.16
                cube.color.a = 0.82
                dist_val = info['mean_distance_m']
                label_text = f'VAGA {spot_id}\nOCUPADA ({dist_val:.2f}m)'
            else:
                cube.color.r = 0.55
                cube.color.g = 0.56
                cube.color.b = 0.60
                cube.color.a = 0.60
                label_text = f'VAGA {spot_id}\nAGUARDANDO'

            marker_array.markers.append(cube)

            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = now_stamp
            text_marker.ns = 'parking_labels'
            text_marker.id = 100 + spot_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = spot_x
            text_marker.pose.position.y = spot_y
            text_marker.pose.position.z = 1.80
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.34
            text_marker.color.r = 1.00
            text_marker.color.g = 1.00
            text_marker.color.b = 1.00
            text_marker.color.a = 1.00
            text_marker.text = label_text

            marker_array.markers.append(text_marker)

        self.markers_pub.publish(marker_array)


def main(args=None) -> None:
    """Ponto de entrada do no central_monitor."""
    rclpy.init(args=args)
    node = CentralMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando central_monitor_node por solicitacao do usuario.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
