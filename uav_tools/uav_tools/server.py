#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0

from enum import IntEnum
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32

# Definición de estados (Misma que tu código original)


class DroneState(IntEnum):
    IDLE = 0
    AUTOMATIC = 1
    TAKING_OFF = 2
    LANDING = 3
    EMERGENCY_STOP = 4


class HeadlessSequencer(Node):
    def __init__(self):
        super().__init__('headless_sequencer')

        # Configuración de QoS (Transient Local es importante para que el
        # comando persista)
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Publicador
        self.state_publisher = self.create_publisher(
            Int32, '/state', qos_profile)
        self.get_logger().info('Nodo secuenciador iniciado (Modo Servidor).')

    def send_command(self, state):
        """Envía el comando al tópico."""
        msg = Int32()
        msg.data = state.value
        self.state_publisher.publish(msg)
        self.get_logger().info(
            f'>>> COMANDO ENVIADO: {state.name} ({state.value})')


def main(args=None):
    rclpy.init(args=args)

    node = HeadlessSequencer()

    try:
        # --- INICIO DE LA SECUENCIA AUTOMÁTICA ---

        # 1. Espera inicial de 5 segundos antes de despegar
        node.get_logger().info('Esperando 5 segundos para el despegue...')
        time.sleep(5)

        # 2. Activar Take Off
        node.send_command(DroneState.TAKING_OFF)

        # 3. Esperar 10 segundos con el Take Off activo
        node.get_logger().info('Despegue iniciado. Esperando 10 segundos para modo Automático...')
        time.sleep(10)

        # 4. Activar modo Automático
        node.send_command(DroneState.AUTOMATIC)
        node.get_logger().info('Modo Automático ACTIVADO. El script permanecerá activo.')

        # --- FIN DE SECUENCIA, MANTENER NODO VIVO ---

        # Hacemos spin para mantener el nodo corriendo y que el QoS Transient Local
        # siga sirviendo el último mensaje a quien se conecte tarde.
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Captura Ctrl+C para salir limpio
        node.get_logger().info('Deteniendo secuenciador...')

        # Opcional: Mandar aterrizaje o parada al cerrar
        # node.send_command(DroneState.LANDING)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
