#!/usr/bin/env python3
"""No ROS2 de classificacao de vagas de estacionamento por distancia media ultrassonica (Opcao C)."""

import collections
from datetime import datetime
import json
import math
import time
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, String


class UltrasonicDetectorNode(Node):
    """Analisa a distancia media medida pelo sensor ultrassonico direito em cada vaga."""

    def __init__(self) -> None:
        super().__init__('ultrasonic_detector_node')

        # Parametros calibrados para vagas de 5.0 m de comprimento
        # Vaga ocupada: frente do veiculo em Y=-1.4 m -> leitura ~1.22 m
        # Vaga livre: parede de fundo em Y=-6.2 m -> leitura ~6.02 m
        self.declare_parameter('occupancy_threshold_m', 4.2)
        self.declare_parameter('max_effective_range_m', 7.0)
        self.declare_parameter('min_valid_range_m', 0.05)
        self.declare_parameter('stabilization_wait_sec', 1.5)
        self.declare_parameter('buffer_size', 15)

        self.occupancy_threshold_m: float = float(
            self.get_parameter('occupancy_threshold_m').value
        )
        self.max_effective_range_m: float = float(
            self.get_parameter('max_effective_range_m').value
        )
        self.min_valid_range_m: float = float(
            self.get_parameter('min_valid_range_m').value
        )
        self.stabilization_wait_sec: float = float(
            self.get_parameter('stabilization_wait_sec').value
        )
        buffer_maxlen = int(self.get_parameter('buffer_size').value)

        # Buffer circular das leituras recentes (padrao maxlen=15)
        self.readings_buffer: Deque[float] = collections.deque(maxlen=buffer_maxlen)
        self.latest_clean_reading: Optional[float] = None

        # Controle nao-bloqueante de inspecao sob demanda
        self.pending_spot_id: Optional[int] = None
        self.trigger_start_sec: float = 0.0
        self.using_ros_clock: bool = False

        # Assinaturas e Publicacoes
        self.range_sub = self.create_subscription(
            Range,
            '/ultrasonic/right',
            self._right_ultrasonic_callback,
            25,
        )
        self.trigger_sub = self.create_subscription(
            Int32,
            '/inspection_trigger',
            self._inspection_trigger_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, '/spot_status_update', 10)

        # Timer de verificacao da janela de coleta estavel (20 Hz)
        self.eval_timer = self.create_timer(0.05, self._evaluate_pending_inspection)

        self.get_logger().info(
            f'No ultrasonic_detector iniciado | Limiar de ocupacao: '
            f'{self.occupancy_threshold_m:.2f} m | Buffer circular: {buffer_maxlen} amostras'
        )

    def _now_sec(self) -> float:
        """Retorna o instante atual em segundos usando o clock ROS2/Gazebo quando ativo."""
        ros_ns = self.get_clock().now().nanoseconds
        if ros_ns > 0:
            now_val = ros_ns * 1e-9
            if not self.using_ros_clock:
                self.using_ros_clock = True
                if self.pending_spot_id is not None:
                    self.trigger_start_sec = now_val
            return now_val
        return time.monotonic()

    def _right_ultrasonic_callback(self, msg: Range) -> None:
        """Filtra leituras do ultrassom lateral e alimenta o buffer circular."""
        raw_value = float(msg.range)
        if math.isnan(raw_value):
            return

        sensor_max = (
            float(msg.max_range)
            if msg.max_range > 0.0
            else self.max_effective_range_m
        )
        sensor_min = max(self.min_valid_range_m, float(msg.min_range))

        # Valores inf ou acima de max_range sao tratados como 7.0 m
        if (
            math.isinf(raw_value)
            or raw_value > sensor_max
            or raw_value > self.max_effective_range_m
        ):
            clean_distance = self.max_effective_range_m
        elif raw_value < sensor_min:
            return
        else:
            clean_distance = raw_value

        self.latest_clean_reading = clean_distance
        self.readings_buffer.append(clean_distance)

    def _inspection_trigger_callback(self, msg: Int32) -> None:
        """Inicia janela de 1.5s de amostragem estavel com o robo parado na vaga."""
        spot_id = int(msg.data)
        self.pending_spot_id = spot_id
        self.trigger_start_sec = self._now_sec()
        # Limpa amostras antigas colhidas em movimento para garantir precisao estatica
        self.readings_buffer.clear()
        self.get_logger().info(
            f'Gatilho recebido para Vaga {spot_id}: aguardando '
            f'{self.stabilization_wait_sec:.1f}s de amostras estaveis do ultrassom.'
        )

    def _evaluate_pending_inspection(self) -> None:
        """Processa media (mu) e desvio padrao (sigma) apos completar a janela estavel."""
        if self.pending_spot_id is None:
            return

        elapsed = self._now_sec() - self.trigger_start_sec
        if elapsed < self.stabilization_wait_sec:
            return

        spot_id = self.pending_spot_id
        self.pending_spot_id = None

        samples = list(self.readings_buffer)
        if not samples and self.latest_clean_reading is not None:
            samples = [self.latest_clean_reading]

        if not samples:
            self.get_logger().error(
                f'Nenhuma leitura ultrassonica disponivel para classificar a Vaga {spot_id}!'
            )
            return

        sample_count = len(samples)
        mean_distance_mu = sum(samples) / float(sample_count)
        variance = sum((val - mean_distance_mu) ** 2 for val in samples) / float(sample_count)
        std_dev_sigma = math.sqrt(variance)

        if mean_distance_mu < self.occupancy_threshold_m:
            status_str = 'OCUPADA'
        else:
            status_str = 'LIVRE'

        payload = {
            'spot_id': spot_id,
            'status': status_str,
            'mean_distance_m': round(mean_distance_mu, 3),
            'std_dev_m': round(std_dev_sigma, 3),
            'samples': sample_count,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
        }

        out_msg = String()
        out_msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(out_msg)

        self.get_logger().info(
            f'[VAGA {spot_id}] Status: {status_str} | '
            f'Media (mu): {mean_distance_mu:.3f} m | '
            f'Desvio (sigma): {std_dev_sigma:.3f} m | '
            f'Amostras: {sample_count}'
        )


def main(args=None) -> None:
    """Ponto de entrada do no ultrasonic_detector."""
    rclpy.init(args=args)
    node = UltrasonicDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando ultrasonic_detector_node por solicitacao do usuario.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
