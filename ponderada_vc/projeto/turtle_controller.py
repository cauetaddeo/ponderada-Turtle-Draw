import ast
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

FUNCOES_VISAO = ("gaussian_kernel", "convolve", "sobel")

QUEBRA_TRAJETO = None

VIZINHOS_PIXEL = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
]


def carregar_funcoes_visao():
    caminho_funcoes = Path(__file__).with_name("funcoes_visao.py")

    arvore = ast.parse(caminho_funcoes.read_text(encoding="utf-8-sig"))
    definicoes = [
        no for no in arvore.body
        if isinstance(no, ast.FunctionDef) and no.name in FUNCOES_VISAO
    ]

    namespace = {"np": np}
    modulo = ast.Module(body=definicoes, type_ignores=[])
    ast.fix_missing_locations(modulo)
    exec(compile(modulo, str(caminho_funcoes), "exec"), namespace)

    return tuple(namespace[nome] for nome in FUNCOES_VISAO)


def carregar_imagem_cinza(caminho_img):
    img_bgr = cv2.imread(str(caminho_img))
    if img_bgr is None:
        raise FileNotFoundError(f"Nao foi possivel carregar a imagem: {caminho_img}")

    img_bgr = img_bgr.astype(np.float64)
    return 0.114 * img_bgr[:, :, 0] + 0.587 * img_bgr[:, :, 1] + 0.299 * img_bgr[:, :, 2]


def componentes_conectados(binary, tamanho_minimo=10):
    mascara = binary > 0
    visitado = np.zeros_like(mascara, dtype=bool)
    altura, largura = mascara.shape
    componentes = []

    for linha in range(altura):
        for coluna in range(largura):
            if not mascara[linha, coluna] or visitado[linha, coluna]:
                continue

            fila = deque([(linha, coluna)])
            visitado[linha, coluna] = True
            componente = []

            while fila:
                atual_linha, atual_coluna = fila.popleft()
                componente.append((atual_linha, atual_coluna))

                for d_linha, d_coluna in VIZINHOS_PIXEL:
                    prox_linha = atual_linha + d_linha
                    prox_coluna = atual_coluna + d_coluna

                    if not (0 <= prox_linha < altura and 0 <= prox_coluna < largura):
                        continue
                    if visitado[prox_linha, prox_coluna] or not mascara[prox_linha, prox_coluna]:
                        continue

                    visitado[prox_linha, prox_coluna] = True
                    fila.append((prox_linha, prox_coluna))

            if len(componente) >= tamanho_minimo:
                componentes.append(componente)

    return componentes


def criar_grafo_adjacencia(pontos):
    pontos_set = set(pontos)
    grafo = {}

    for linha, coluna in pontos_set:
        vizinhos = []
        for d_linha, d_coluna in VIZINHOS_PIXEL:
            candidato = (linha + d_linha, coluna + d_coluna)
            if candidato in pontos_set:
                vizinhos.append(candidato)
        grafo[(linha, coluna)] = vizinhos

    return grafo


def grau_restante(grafo, restantes, ponto):
    return sum(1 for vizinho in grafo[ponto] if vizinho in restantes)


def escolher_inicio_traco(grafo, restantes):
    pontas = [ponto for ponto in restantes if grau_restante(grafo, restantes, ponto) <= 1]
    candidatos = pontas if pontas else restantes
    return min(candidatos, key=lambda p: (p[0], p[1]))


def escolher_proximo_ponto(grafo, restantes, atual, anterior):
    candidatos = [ponto for ponto in grafo[atual] if ponto in restantes]
    if not candidatos:
        return None

    if anterior is None:
        return min(candidatos, key=lambda p: grau_restante(grafo, restantes, p))

    direcao_linha = atual[0] - anterior[0]
    direcao_coluna = atual[1] - anterior[1]

    def custo(ponto):
        prox_linha = ponto[0] - atual[0]
        prox_coluna = ponto[1] - atual[1]
        alinhamento = direcao_linha * prox_linha + direcao_coluna * prox_coluna
        return (-alinhamento, grau_restante(grafo, restantes, ponto), ponto[0], ponto[1])

    return min(candidatos, key=custo)


def decompor_em_tracos_locais(componente, tamanho_minimo=4):
    grafo = criar_grafo_adjacencia(componente)
    restantes = set(componente)
    tracos = []

    while restantes:
        atual = escolher_inicio_traco(grafo, restantes)
        anterior = None
        traco = []

        while atual in restantes:
            traco.append(atual)
            restantes.remove(atual)

            proximo = escolher_proximo_ponto(grafo, restantes, atual, anterior)
            if proximo is None:
                break

            anterior, atual = atual, proximo

        if len(traco) >= tamanho_minimo:
            tracos.append(traco)

    return tracos


def mapear_para_turtlesim(pontos_pixel, caixa, margem=1.0):
    linha_min, linha_max, coluna_min, coluna_max = caixa
    altura_bbox = max(1, linha_max - linha_min)
    largura_bbox = max(1, coluna_max - coluna_min)

    area = 11.0 - 2 * margem
    escala = area / max(altura_bbox, largura_bbox)
    largura_desenho = largura_bbox * escala
    altura_desenho = altura_bbox * escala
    desloc_x = margem + (area - largura_desenho) / 2
    desloc_y = margem + (area - altura_desenho) / 2

    caminho = []
    for linha, coluna in pontos_pixel:
        x = desloc_x + (coluna - coluna_min) * escala
        y = 11.0 - (desloc_y + (linha - linha_min) * escala)
        caminho.append((float(x), float(y)))

    return caminho


def planejar_caminho_turtlesim(binary, limite_pontos=2200):
    pontos = np.argwhere(binary > 0)
    if len(pontos) == 0:
        return []

    linha_min, coluna_min = pontos.min(axis=0)
    linha_max, coluna_max = pontos.max(axis=0)
    caixa = (int(linha_min), int(linha_max), int(coluna_min), int(coluna_max))

    componentes = componentes_conectados(binary)
    componentes.sort(key=len, reverse=True)

    tracos = []
    for componente in componentes:
        tracos.extend(decompor_em_tracos_locais(componente))

    tracos.sort(key=len, reverse=True)
    total = sum(len(traco) for traco in tracos)
    passo_global = max(1, math.ceil(total / limite_pontos))

    caminho = []
    for traco in tracos:
        traco_reduzido = traco[::passo_global]
        if len(traco_reduzido) < 2:
            continue

        if caminho:
            caminho.append(QUEBRA_TRAJETO)
        caminho.extend(mapear_para_turtlesim(traco_reduzido, caixa))

    if caminho:
        caminho.insert(0, QUEBRA_TRAJETO)

    return caminho


class TurtleDrawNode(Node):

    def __init__(self):
        super().__init__('turtle_draw_node')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        self.pose_atual = None
        self.caminho_pontos = []
        self.ponto_atual_idx = 0

        self.declare_parameter(
            'imagem',
            str(Path(__file__).resolve().parents[1] / 'dog.jpg'),
        )
        caminho_img = self.get_parameter('imagem').value

        self.get_logger().info('Processando imagem com as funcoes atuais de funcoes_visao.py...')
        gaussian_kernel, convolve, sobel = carregar_funcoes_visao()

        img_cinza = carregar_imagem_cinza(caminho_img)
        img_suavizada = convolve(img_cinza, gaussian_kernel())
        img_bordas = sobel(img_suavizada)
        binary = (img_bordas > 80).astype(np.uint8) * 255

        self.caminho_pontos = planejar_caminho_turtlesim(binary)

        pontos_reais = sum(1 for ponto in self.caminho_pontos if ponto is not QUEBRA_TRAJETO)
        self.get_logger().info(f'Processamento concluido: {pontos_reais} pontos planejados.')

        self.timer = self.create_timer(0.015, self.mover_tartaruga)

    def pose_callback(self, msg):
        self.pose_atual = msg

    def configurar_caneta(self, desligada):
        if not self.pen_client.service_is_ready():
            return

        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 1
        req.off = 1 if desligada else 0
        self.pen_client.call_async(req)

    def teleportar(self, x, y):
        if not self.teleport_client.service_is_ready():
            return False

        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0
        self.teleport_client.call_async(req)
        return True

    def pular_para_proximo_contorno(self):
        self.publisher_.publish(Twist())
        self.ponto_atual_idx += 1

        while self.ponto_atual_idx < len(self.caminho_pontos):
            proximo = self.caminho_pontos[self.ponto_atual_idx]
            if proximo is not QUEBRA_TRAJETO:
                self.configurar_caneta(desligada=True)
                self.teleportar(proximo[0], proximo[1])
                self.configurar_caneta(desligada=False)
                self.ponto_atual_idx += 1
                return
            self.ponto_atual_idx += 1

    def mover_tartaruga(self):
        if self.pose_atual is None:
            return

        if self.ponto_atual_idx >= len(self.caminho_pontos):
            self.get_logger().info('Desenho concluido!')
            self.publisher_.publish(Twist())
            self.timer.cancel()
            return

        alvo = self.caminho_pontos[self.ponto_atual_idx]
        if alvo is QUEBRA_TRAJETO:
            self.pular_para_proximo_contorno()
            return

        alvo_x, alvo_y = alvo
        dx = alvo_x - self.pose_atual.x
        dy = alvo_y - self.pose_atual.y
        distancia = math.sqrt(dx**2 + dy**2)
        angulo_alvo = math.atan2(dy, dx)
        erro_angulo = angulo_alvo - self.pose_atual.theta

        erro_angulo = math.atan2(math.sin(erro_angulo), math.cos(erro_angulo))

        msg = Twist()

        if distancia <= 0.09:
            self.ponto_atual_idx += 1
            return

        if abs(erro_angulo) > 0.22:
            msg.angular.z = max(-5.0, min(5.0, 7.0 * erro_angulo))
            msg.linear.x = 0.0
        else:
            msg.angular.z = max(-2.0, min(2.0, 2.8 * erro_angulo))
            msg.linear.x = min(2.4, max(0.35, 2.6 * distancia))

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDrawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()