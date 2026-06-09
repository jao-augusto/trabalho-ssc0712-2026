#!/usr/bin/env python3
import numpy as np
import cv2
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from enum import Enum


# ---------------------------------------------------------------------------
#       constantes usadas
# ---------------------------------------------------------------------------

# velocidades
VEL_LENTA          = 0.35
VEL_RAPIDA         = 0.9
VEL_DESVIO         = 0.35
VEL_ANGULAR_DESVIO = 0.35
VEL_APROXIMACAO    = 0.1
VEL_ANGULAR_MAX    = 1.0
GANHO_DIRECAO      = 1.0

# lidar
LIMITE_OBSTACULO         = 1.0
LIMITE_OBSTACULO_RAPIDO  = 1.5
LIMITE_OBSTACULO_LATERAL = 0.3
RAIO_CORRIDA             = 2.0
FOV_FRENTE_RAD           = np.radians(33)

# camera e bandeira
COR_BANDEIRA_BGR      = np.array([227, 73, 0])
PIXELS_BANDEIRA_MIN   = 5
PIXELS_CHEGOU         = 5000
LIMIAR_ALINHADO_CAM   = 0.05   # erro normalizado da camera para considerar centralizado
LIMIAR_ALINHADO_LIDAR = 0.02   # erro angular (rad) do lidar para considerar centralizado
DIST_PEGAR_BANDEIRA   = 0.21 + 0.2 + 0.1 + 0.06  # base + pole + palma + dedo

# tempos
TEMPO_CONFIRMAR_BANDEIRA = 3.0   # segundos parado para confirmar que e a bandeira
TEMPO_ESPERA_FASE        = 2.0   # segundos de pausa entre fases do alinhamento final


# ---------------------------------------------------------------------------
#       estados
# ---------------------------------------------------------------------------

class Estado(Enum):
    VAGANDO           = 'vagando'
    BANDEIRA_AVISTADA = 'bandeira_avistada'
    NAVEGANDO         = 'navegando'
    CHEGOU            = 'chegou'


# ---------------------------------------------------------------------------
#       no de controle
# ---------------------------------------------------------------------------

class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)

        self.create_subscription(LaserScan, '/scan',                  self._scan_callback,   10)
        self.create_subscription(Imu,       '/imu',                   self._imu_callback,    10)
        self.create_subscription(Odometry,  '/odom',                  self._odom_callback,   10)
        self.create_subscription(Image,     '/robot_cam/colored_map', self._camera_callback, 10)

        self.bridge = CvBridge()
        self.timer  = self.create_timer(0.1, self._loop_controle)

        # maquina de estados
        self.estado = Estado.VAGANDO

        # lidar
        self.obstaculo_a_frente = False
        self.acelerar           = False
        self.direcao_desvio     = -1.0
        self._ultimo_scan       = None

        # camera
        self.bandeira_visivel         = False
        self.direcao_bandeira         = 0.0   # -1 (esq) a +1 (dir)
        self._ultima_direcao_bandeira = 0.0   # ultimo valor antes de sumir

        # navegacao
        self.angulo_bandeira = None   # angulo absoluto (world frame) para a bandeira
        self.yaw_atual       = 0.0

        # confirmacao da bandeira
        self.tempo_avistou_bandeira = None

        # fases do alinhamento final
        self.fase_chegou = 1   # 1 = camera, 2 = lidar, 3 = avanca


    # ------------------------------------------------------------------
    #            callbacks
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        obstaculo, angulo_obstaculo = self._analisar_lidar(msg)

        if obstaculo:
            self.obstaculo_a_frente = True
            self.acelerar           = False
            self.direcao_desvio     = -1.0 if angulo_obstaculo < np.pi else 1.0
        else:
            self.obstaculo_a_frente = False
            self.acelerar           = self._caminho_livre(msg)

        self._ultimo_scan = msg

    def _analisar_lidar(self, msg: LaserScan):
        """Retorna (ha_obstaculo, angulo_do_obstaculo_mais_proximo)"""
        limite = LIMITE_OBSTACULO_RAPIDO if self.acelerar else LIMITE_OBSTACULO

        min_dist  = float('inf')
        min_angle = 0.0

        for i, dist in enumerate(msg.ranges):
            if not (msg.range_min < dist < msg.range_max):
                continue

            ang      = msg.angle_min + i * msg.angle_increment
            ang_norm = (ang + np.pi) % (2 * np.pi) - np.pi

            if abs(ang_norm) > np.pi / 2:
                continue

            limite_atual = limite if abs(ang_norm) <= FOV_FRENTE_RAD else LIMITE_OBSTACULO_LATERAL

            if dist < limite_atual and dist < min_dist:
                min_dist  = dist
                min_angle = ang

        return (min_dist < float('inf'), min_angle)

    def _caminho_livre(self, msg: LaserScan) -> bool:
        """vdd se a frente ta livre o suficiente para acelerar"""
        leituras = [
            dist for i, dist in enumerate(msg.ranges)
            if msg.range_min < dist < msg.range_max
            and abs(((msg.angle_min + i * msg.angle_increment) + np.pi) % (2 * np.pi) - np.pi) <= FOV_FRENTE_RAD
        ]
        return bool(leituras) and min(leituras) > RAIO_CORRIDA

    def _imu_callback(self, msg: Imu):
        pass # nao uso imu

    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw_atual = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2] # usa so o yaw

    def _camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('camera', frame)
        cv2.waitKey(1)

        mask   = cv2.inRange(frame, COR_BANDEIRA_BGR, COR_BANDEIRA_BGR)
        pixels = cv2.countNonZero(mask)

        if pixels > PIXELS_CHEGOU:
            self.bandeira_visivel = True
            self._ir_para(Estado.CHEGOU, 'Chegou na bandeira!')

        elif pixels > PIXELS_BANDEIRA_MIN:
            coords = np.column_stack(np.where(mask > 0))
            cx = int(np.mean(coords[:, 1]))
            self.direcao_bandeira = (cx - frame.shape[1] / 2) / (frame.shape[1] / 2)
            self.bandeira_visivel = True

            # atualiza angulo absoluto da bandeira so quando nao tem obstaculo na frente
            # (evita usar a distancia de um cilindro como referencia)
            if not self.obstaculo_a_frente and pixels > PIXELS_BANDEIRA_MIN:
                self.angulo_bandeira = self.yaw_atual + self.direcao_bandeira * FOV_FRENTE_RAD

            if self.estado == Estado.VAGANDO:
                self._ir_para(Estado.BANDEIRA_AVISTADA, 'Bandeira avistada! Confirmando por 3s...')
                self.tempo_avistou_bandeira = self.get_clock().now()

        else:
            self.bandeira_visivel = False

            # guarda o lado em que a bandeira sumiu para saber pra onde girar
            if self.estado == Estado.NAVEGANDO:
                self._ultima_direcao_bandeira = self.direcao_bandeira


    # ------------------------------------------------------------------
    #        loop principal
    # ------------------------------------------------------------------

    def _loop_controle(self):
        twist = self._twist_vazio()

        # checa se os 3s de confirmação ja passaram
        if self.estado == Estado.BANDEIRA_AVISTADA and self.tempo_avistou_bandeira is not None:
            elapsed = (self.get_clock().now() - self.tempo_avistou_bandeira).nanoseconds / 1e9
            if elapsed >= TEMPO_CONFIRMAR_BANDEIRA:
                self._ir_para(Estado.NAVEGANDO, 'Confirmado! Indo para a bandeira...')

        acoes = {
            Estado.VAGANDO:           self._vagando,
            Estado.BANDEIRA_AVISTADA: self._bandeira_avistada,
            Estado.NAVEGANDO:         self._navegando,
            Estado.CHEGOU:            self._chegou,
        }
        acoes[self.estado](twist)

        self.get_logger().info(f'Estado: [{self.estado.value}]')
        self.cmd_vel_pub.publish(twist)

    def _twist_vazio(self) -> TwistStamped:
        twist = TwistStamped()
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        return twist


    # ------------------------------------------------------------------
    #             comportamentos
    # ------------------------------------------------------------------

    def _vagando(self, twist):
        """explora o ambiente vagando por ai enquanto desvia de obstaculos"""
        if self.obstaculo_a_frente:
            twist.twist.linear.x  = VEL_DESVIO
            twist.twist.angular.z = self.direcao_desvio * VEL_ANGULAR_DESVIO
        else:
            twist.twist.linear.x = VEL_RAPIDA if self.acelerar else VEL_LENTA

    def _bandeira_avistada(self, twist):
        """serve so pra parar o robo por uns 3 segundos ao avistar a bandeira"""
        pass  # twist zerado = robô parado

    def _navegando(self, twist):
        """ele tem um angulo que gravou (onde ta a bandeira) e segue esse angulo como uma direcao guia
        se perder de vista, vira pro lado onde a bandeira tava"""
        if self.obstaculo_a_frente:
            twist.twist.linear.x  = VEL_DESVIO
            twist.twist.angular.z = self.direcao_desvio * VEL_ANGULAR_DESVIO
            return

        if self.bandeira_visivel:
            if self.angulo_bandeira is None:
                twist.twist.linear.x = VEL_LENTA  # anda reto ate ter angulo confiavel
                return
            erro = self.angulo_bandeira - self.yaw_atual
            erro = (erro + np.pi) % (2 * np.pi) - np.pi
            twist.twist.linear.x  = VEL_RAPIDA if self.acelerar else VEL_LENTA
            twist.twist.angular.z = np.clip(-erro * GANHO_DIRECAO, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX)
        else:
            # bandeira sumiu, entao para se tiver algo perto e anda devagar se estiver longe
            dist_frente = self._distancia_frente()
            twist.twist.linear.x  = 0.0 if dist_frente < 1.0 else VEL_LENTA
            twist.twist.angular.z = -np.sign(self._ultima_direcao_bandeira) * VEL_ANGULAR_DESVIO

    def _chegou(self, twist):
        """alinha o robo com o cabo da bandeira para a futura tarefa de captura-la

        fase 1 — camera: centraliza a bandeira na imagem
        fase 2 — lidar:  centraliza o ponto mais proximo no angulo 0
        fase 3 — avanca: vai devagar ate a distancia de pegar
        """
        if self.fase_chegou == 1:
            if not self.bandeira_visivel:
                return
            if abs(self.direcao_bandeira) > LIMIAR_ALINHADO_CAM:
                twist.twist.angular.z = -self.direcao_bandeira * GANHO_DIRECAO * 0.5
            else:
                self.get_logger().info('Câmera alinhada, esperando 2s...')
                time.sleep(TEMPO_ESPERA_FASE)
                self.fase_chegou = 2

        elif self.fase_chegou == 2:
            erro = self._erro_angular_lidar()
            if abs(erro) > LIMIAR_ALINHADO_LIDAR:
                twist.twist.angular.z = np.clip(erro * GANHO_DIRECAO, -VEL_ANGULAR_MAX, VEL_ANGULAR_MAX)
            else:
                self.get_logger().info('Lidar alinhado, esperando 2s...')
                time.sleep(TEMPO_ESPERA_FASE)
                self.fase_chegou = 3

        elif self.fase_chegou == 3:
            dist = self._distancia_frente()
            if dist > DIST_PEGAR_BANDEIRA:
                twist.twist.linear.x = VEL_APROXIMACAO
            else:
                self.get_logger().info('Gripper posicionado. Pronto para pegar!')


    # ------------------------------------------------------------------
    #      helpers do lidar
    # ------------------------------------------------------------------

    def _distancia_frente(self) -> float:
        """retorna a menor distancia dentro do fov"""
        if self._ultimo_scan is None:
            return float('inf')
        msg = self._ultimo_scan
        leituras = [
            dist for i, dist in enumerate(msg.ranges)
            if msg.range_min < dist < msg.range_max
            and abs(((msg.angle_min + i * msg.angle_increment) + np.pi) % (2 * np.pi) - np.pi) <= FOV_FRENTE_RAD
        ]
        return min(leituras) if leituras else float('inf')

    def _erro_angular_lidar(self) -> float:
        """retorna o angulo do ponto mais proximo dentro do fov.
        e usado pra alinhar o robo a (com crase) bandeira
        """
        if self._ultimo_scan is None:
            return 0.0
        msg = self._ultimo_scan

        min_dist  = float('inf')
        min_angle = 0.0

        for i, dist in enumerate(msg.ranges):
            if not (msg.range_min < dist < msg.range_max):
                continue
            ang_norm = ((msg.angle_min + i * msg.angle_increment) + np.pi) % (2 * np.pi) - np.pi
            if abs(ang_norm) > FOV_FRENTE_RAD:
                continue
            if dist < min_dist:
                min_dist  = dist
                min_angle = ang_norm

        return min_angle


    # ------------------------------------------------------------------
    #      utilidades
    # ------------------------------------------------------------------

    def _ir_para(self, novo_estado: Estado, mensagem: str):
        if self.estado == novo_estado:
            return
        if novo_estado == Estado.CHEGOU:
            self.fase_chegou = 1
        self.get_logger().info(f'[{self.estado.value}] → [{novo_estado.value}] | {mensagem}')
        self.estado = novo_estado


# ---------------------------------------------------------------------------
#       main
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()