# Turtle Draw com ROS 2 e Visao Computacional


## Vídeo do projeto: https://youtu.be/Eo6Mn5gZoXQ

## Introducao

Este projeto implementa uma pipeline completa de visao computacional para transformar uma imagem em contornos e fazer a tartaruga do `turtlesim` desenhar esses contornos na tela. A proposta segue o enunciado da atividade: carregar uma imagem, processa-la do zero, detectar bordas, planejar um caminho no espaco do `turtlesim` e controlar a tartaruga usando ROS 2.

A imagem usada no projeto e `dog.jpg`. A pipeline primeiro converte a imagem para escala de cinza, aplica suavizacao Gaussiana, detecta bordas com Sobel, limiariza o resultado e transforma os pixels de borda em trajetorias para a tartaruga.

Um ponto importante do projeto e que o OpenCV foi usado apenas para carregar a imagem com `cv2.imread`. As demais etapas de processamento visual foram feitas manualmente com NumPy, respeitando o requisito tecnico do enunciado.

## Estrutura principal

Os arquivos principais sao:

```text
ponderada_vc/
├── dog.jpg
└── projeto/
    ├── funcoes_visao.py
    └── turtle_controller.py
```

- `funcoes_visao.py`: responsavel pela pipeline de visao computacional e pela visualizacao das etapas da imagem.
- `turtle_controller.py`: responsavel por usar as funcoes de visao, planejar os pontos no espaco do turtlesim e controlar a tartaruga via ROS 2.

## funcoes_visao.py

O arquivo `funcoes_visao.py` implementa a parte de visao computacional.

### Carregamento da imagem

```python
caminho_img = Path(__file__).resolve().parents[1] / 'dog.jpg'
img_bgr = cv2.imread(str(caminho_img))
```

O caminho da imagem e calculado com base na localizacao do proprio arquivo. Isso evita erro ao executar o script a partir de outro diretorio. Antes, o codigo usava apenas `dog.jpg`, o que fazia o OpenCV procurar a imagem no diretorio atual do terminal.

### Conversao para escala de cinza

```python
img_bgr = img_bgr.astype(np.float64)
img = 0.114 * img_bgr[:, :, 0] + 0.587 * img_bgr[:, :, 1] + 0.299 * img_bgr[:, :, 2]
```

A imagem carregada pelo OpenCV vem no formato BGR. A conversao para cinza foi feita manualmente com NumPy, usando a combinacao ponderada dos canais azul, verde e vermelho. Isso substitui o uso de `cv2.cvtColor`, porque o enunciado permite OpenCV apenas para leitura da imagem.

### gaussian_kernel

```python
def gaussian_kernel(size=5, sigma=1.4):
```

Essa funcao cria um kernel Gaussiano 2D. O objetivo e suavizar a imagem antes da deteccao de bordas. A suavizacao reduz ruidos e pequenos detalhes que poderiam gerar bordas falsas.

A funcao usa:

- `np.arange` para criar os eixos do kernel;
- `np.meshgrid` para montar a grade 2D;
- a formula da Gaussiana para calcular os pesos;
- normalizacao para garantir que a soma do kernel seja igual a 1.

### convolve

```python
def convolve(image, kernel):
```

Essa funcao aplica convolucao 2D manualmente. Ela percorre cada pixel da imagem, pega uma regiao ao redor dele e calcula a soma ponderada pelos valores do kernel.

O padding usado e `reflect`, ou seja, as bordas da imagem sao espelhadas. Isso evita perder informacao nas extremidades e permite aplicar o filtro em todos os pixels.

### sobel

```python
def sobel(image):
```

A funcao `sobel` implementa o detector de bordas de Sobel do zero. Ela usa dois kernels:

```python
Kx = [[-1, 0, 1],
      [-2, 0, 2],
      [-1, 0, 1]]

Ky = [[-1, -2, -1],
      [ 0,  0,  0],
      [ 1,  2,  1]]
```

O kernel `Kx` detecta variacoes horizontais e o kernel `Ky` detecta variacoes verticais. Depois disso, a magnitude do gradiente e calculada com:

```python
magnitude = np.sqrt(Gx**2 + Gy**2)
```

Por fim, o resultado e normalizado para o intervalo de 0 a 255.

### Limiarizacao

```python
threshold = 80
binary = (edges > threshold).astype(np.uint8) * 255
```

A limiarizacao transforma a imagem de bordas em uma imagem binaria. Pixels com intensidade maior que 80 viram branco, representando contorno. Os demais viram preto, representando fundo.

### Visualizacao

O script mostra tres etapas:

1. Imagem em escala de cinza.
2. Bordas detectadas com Sobel.
3. Resultado binario apos limiarizacao.

Tambem salva a visualizacao em:

```text
ponderada_vc/projeto/resultado_visao.png
```

Para executar apenas a parte de visao:

```bash
cd ~/workspace_visao_comp/src/ponderada_vc/ponderada_vc/projeto
python3 funcoes_visao.py
```

Se estiver sem interface grafica disponivel:

```bash
MPLBACKEND=Agg python3 funcoes_visao.py
```

## turtle_controller.py

O arquivo `turtle_controller.py` implementa o no ROS 2 que controla a tartaruga.

### Importacao das funcoes de visao

```python
FUNCOES_VISAO = ("gaussian_kernel", "convolve", "sobel")
```

O controller usa as funcoes atuais de `funcoes_visao.py`: `gaussian_kernel`, `convolve` e `sobel`.

Como `funcoes_visao.py` tambem executa visualizacao com Matplotlib no nivel global, o controller nao importa o arquivo diretamente. Em vez disso, ele le o arquivo com `ast`, pega apenas as definicoes das funcoes necessarias e compila essas funcoes em memoria.

```python
arvore = ast.parse(caminho_funcoes.read_text(encoding="utf-8-sig"))
```

O encoding `utf-8-sig` e usado para evitar erro com o caractere BOM (`U+FEFF`), que pode aparecer quando arquivos sao editados no Windows.

### carregar_imagem_cinza

```python
def carregar_imagem_cinza(caminho_img):
```

Essa funcao carrega a imagem com `cv2.imread` e converte para cinza usando NumPy. Assim como em `funcoes_visao.py`, o OpenCV fica restrito ao carregamento da imagem.

### componentes_conectados

```python
def componentes_conectados(binary, tamanho_minimo=10):
```

Essa funcao encontra grupos de pixels brancos conectados na imagem binaria. Cada grupo representa um pedaco de contorno. A busca considera os 8 vizinhos de cada pixel:

```python
VIZINHOS_PIXEL = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
]
```

O objetivo e separar o desenho em partes menores, evitando que a tartaruga conecte pontos distantes e risque linhas atravessando o cachorro.

### Grafo de adjacencia

```python
def criar_grafo_adjacencia(pontos):
```

Depois de encontrar um componente, o controller cria um grafo onde cada pixel branco e um no, e seus vizinhos brancos sao as conexoes. Isso permite transformar os pixels em pequenos tracos locais.

### decompor_em_tracos_locais

```python
def decompor_em_tracos_locais(componente, tamanho_minimo=4):
```

Essa funcao separa um componente em varios tracos. A tartaruga so conecta pixels adjacentes. Quando nao existe mais um vizinho valido, o traco termina. Isso evita sobreposicoes grandes e linhas diagonais artificiais.

### mapear_para_turtlesim

```python
def mapear_para_turtlesim(pontos_pixel, caixa, margem=1.0):
```

Essa funcao transforma coordenadas de pixel da imagem em coordenadas do `turtlesim`, cujo espaco vai aproximadamente de 0 a 11 nos eixos X e Y.

O mapeamento:

- calcula a caixa delimitadora do cachorro;
- preserva a proporcao da imagem;
- centraliza o desenho na tela;
- inverte o eixo Y, porque imagens crescem para baixo e o turtlesim cresce para cima.

### planejar_caminho_turtlesim

```python
def planejar_caminho_turtlesim(binary, limite_pontos=2200):
```

Essa e a funcao principal de planejamento. Ela recebe a imagem binaria de bordas e retorna uma lista de pontos para a tartaruga seguir.

O parametro `limite_pontos` controla o nivel de detalhe:

- valores maiores geram mais detalhes, mas deixam o desenho mais lento;
- valores menores deixam o desenho mais rapido, mas simplificam o contorno.

Exemplos:

```python
limite_pontos=1500  # mais rapido e mais simples
limite_pontos=2200  # equilibrio atual
limite_pontos=3200  # mais detalhado e mais lento
```

Entre tracos separados, o controller insere `QUEBRA_TRAJETO`. Quando encontra essa quebra, a tartaruga levanta a caneta e teleporta para o proximo trecho.

### TurtleDrawNode

```python
class TurtleDrawNode(Node):
```

Essa classe e o no ROS 2 principal. Ela cria:

- um publisher para `/turtle1/cmd_vel`, usado para enviar velocidades;
- um subscriber para `/turtle1/pose`, usado para saber onde a tartaruga esta;
- um client para `/turtle1/set_pen`, usado para ligar e desligar a caneta;
- um client para `/turtle1/teleport_absolute`, usado para mover a tartaruga entre tracos sem desenhar.

### Controle de movimento

A funcao `mover_tartaruga` calcula a distancia e o angulo ate o proximo ponto.

Se a tartaruga ainda nao esta apontada para o alvo, ela gira parada:

```python
if abs(erro_angulo) > 0.22:
    msg.angular.z = max(-5.0, min(5.0, 7.0 * erro_angulo))
    msg.linear.x = 0.0
```

Quando ja esta alinhada, ela anda:

```python
msg.angular.z = max(-2.0, min(2.0, 2.8 * erro_angulo))
msg.linear.x = min(2.4, max(0.35, 2.6 * distancia))
```

Esse controle evita loops, porque a tartaruga nao tenta andar muito enquanto ainda esta virando.

## Como executar

### 1. Entrar no workspace

```bash
cd ~/workspace_visao_comp
```

### 2. Compilar o pacote

```bash
colcon build --packages-select ponderada_vc
```

### 3. Carregar o ambiente

Se necessario, carregue primeiro sua distribuicao ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Depois carregue o pacote compilado:

```bash
source install/setup.bash
```

### 4. Abrir o turtlesim

Em um terminal:

```bash
ros2 run turtlesim turtlesim_node
```

### 5. Rodar o controller

Em outro terminal:

```bash
cd ~/workspace_visao_comp
source install/setup.bash
ros2 run ponderada_vc turtle_controller
```

### 6. Limpar a tela do turtlesim

Se quiser apagar desenhos anteriores:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

## Rodar com outra imagem

O controller aceita o caminho da imagem como parametro ROS:

```bash
ros2 run ponderada_vc turtle_controller --ros-args -p imagem:=/caminho/para/imagem.jpg
```

## Ajustes de qualidade e velocidade

O principal ajuste fica em:

```python
def planejar_caminho_turtlesim(binary, limite_pontos=2200):
```

Sugestoes:

```python
limite_pontos=1500  # desenha mais rapido, com menos detalhe
limite_pontos=2200  # configuracao equilibrada
limite_pontos=3200  # mais detalhe, porem mais lento
```

Tambem e possivel ajustar a velocidade na funcao `mover_tartaruga`, nos campos `linear.x` e `angular.z`. Valores maiores aceleram o desenho, mas podem reduzir a precisao do contorno.

## Problemas comuns

### Erro: nao foi possivel carregar dog.jpg

Isso acontece quando a imagem nao esta instalada junto com o pacote. O `setup.py` foi ajustado para incluir `dog.jpg` como dado do pacote.

Depois de qualquer alteracao no pacote, rode:

```bash
cd ~/workspace_visao_comp
colcon build --packages-select ponderada_vc
source install/setup.bash
```

### A janela do turtlesim nao aparece

No WSL com interface grafica, a janela pode abrir atras de outras janelas. Use `Alt + Tab` no Windows e procure por `TurtleSim`. Se necessario, finalize o processo com `Ctrl+C` no terminal e abra novamente:

```bash
ros2 run turtlesim turtlesim_node
```

### O desenho esta lento

Reduza `limite_pontos`, por exemplo para `1500`, e recompile o pacote.

### O desenho esta pouco detalhado

Aumente `limite_pontos`, por exemplo para `3200`, e recompile o pacote.

## Conclusao

O projeto implementa uma cadeia completa de visao computacional e controle robotico: a imagem e carregada, convertida para cinza, suavizada, processada com Sobel, limiarizada e convertida em trajetorias para o `turtlesim`. O controller ROS 2 usa esses pontos para comandar a tartaruga, controlando velocidade, orientacao, caneta e teletransporte entre tracos.

Durante o desenvolvimento, os principais desafios foram transformar uma imagem binaria de bordas em um caminho coerente para a tartaruga e evitar que ela ligasse pontos distantes com linhas artificiais. Para resolver isso, o caminho foi dividido em componentes conectados e depois em tracos locais, fazendo com que a tartaruga siga melhor os contornos reais do cachorro.

O resultado final demonstra a integracao entre processamento de imagens implementado manualmente e controle de movimento em ROS 2, atendendo aos requisitos centrais da atividade.
