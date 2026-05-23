# Documentação técnica, Turtle Draw com ROS 2 e Visão Computacional

## Pipeline de visão computacional

A primeira etapa da pipeline é o carregamento da imagem. No início, o código tentava abrir apenas `dog.jpg`, usando um caminho relativo simples. Isso causava erro quando o script era executado a partir de outro diretório, porque o Python procurava a imagem no local atual do terminal. Para resolver isso, o caminho passou a ser calculado com `Path(__file__)`, ou seja, baseado na localização do próprio arquivo `funcoes_visao.py`. Assim, o script consegue encontrar a imagem de forma mais confiável.

Depois do carregamento, a imagem é convertida para escala de cinza. Como o OpenCV carrega a imagem no formato BGR, a conversão foi feita manualmente pela fórmula ponderada dos canais azul, verde e vermelho. Essa escolha atende ao requisito de não usar funções prontas de processamento do OpenCV, como `cv2.cvtColor`. A fórmula usada dá mais peso ao canal verde, depois ao vermelho e por último ao azul, seguindo uma aproximação comum da percepção humana de luminosidade.

Em seguida, foi aplicada uma suavização Gaussiana. A função `gaussian_kernel` cria um kernel 2D normalizado, e a função `convolve` aplica esse kernel na imagem. Essa etapa reduz ruído e pequenas variações de intensidade que poderiam gerar bordas falsas. A convolução foi implementada manualmente, percorrendo os pixels da imagem e calculando a soma ponderada da vizinhança. Para tratar as bordas da imagem, foi usado padding do tipo `reflect`, que espelha os pixels das extremidades.

A detecção de bordas foi feita com o operador de Sobel. A função `sobel` usa dois kernels, um para detectar variações na direção horizontal e outro para detectar variações na direção vertical. Depois, as duas respostas são combinadas pela magnitude do gradiente. O resultado é normalizado para o intervalo de 0 a 255, o que facilita a visualização e a etapa seguinte.

A última etapa da visão é a limiarização. Foi definido um limiar de 80, e todos os pixels com intensidade acima desse valor viram branco, representando borda. Os demais pixels viram preto, representando fundo. O resultado é uma imagem binária com os contornos principais do cachorro. O script também mostra as três etapas principais, escala de cinza, bordas Sobel e imagem limiarizada, e salva uma imagem de resultado para verificação.

## Planejamento do caminho

A imagem binária possui muitos pixels brancos, mas esses pixels não estão naturalmente organizados como um caminho para a tartaruga seguir. Esse foi um dos principais desafios do projeto. Se os pontos forem percorridos apenas em ordem de linha e coluna, a tartaruga liga partes distantes do desenho e cria rabiscos que não correspondem ao contorno real.

Para melhorar isso, o controller primeiro separa os pixels brancos em componentes conectados. Cada componente representa um grupo de pixels de borda que estão ligados entre si. Isso evita conectar, por exemplo, uma borda da orelha com uma borda distante do corpo. Componentes muito pequenos são ignorados, pois geralmente representam ruído.

Depois, cada componente é transformado em um grafo de adjacência. Nesse grafo, cada pixel é um nó, e os pixels vizinhos são suas conexões. A partir disso, o código divide os componentes em pequenos traços locais. A tartaruga só conecta pixels que são vizinhos de verdade. Quando um traço termina, a caneta é levantada, e a tartaruga é teleportada para o início do próximo traço. Essa decisão reduziu bastante as linhas artificiais e a sobreposição de traços.

Também foi necessário mapear os pixels da imagem para o espaço do `turtlesim`. A imagem usa coordenadas em pixels, com origem no canto superior esquerdo e eixo Y crescendo para baixo. O `turtlesim` usa um plano de aproximadamente 0 a 11, com eixo Y crescendo para cima. Por isso, o mapeamento inverte o eixo Y, preserva a proporção do cachorro e centraliza o desenho na tela. Foi usada uma margem para evitar que a tartaruga tente desenhar muito perto das bordas da janela.

## Controle da tartaruga

O controller foi implementado como um nó ROS 2 chamado `TurtleDrawNode`. Ele publica comandos de velocidade no tópico `/turtle1/cmd_vel`, recebe a pose atual pelo tópico `/turtle1/pose`, usa o serviço `/turtle1/set_pen` para ligar e desligar a caneta, e usa `/turtle1/teleport_absolute` para mover a tartaruga entre traços sem desenhar linhas indesejadas.

A tartaruga segue os pontos planejados usando um controle proporcional simples. Para cada ponto, o código calcula a distância até o alvo e o erro de ângulo entre a orientação atual da tartaruga e a direção do alvo. Se o erro angular for grande, a tartaruga gira parada. Quando ela já está alinhada, ela anda para frente e faz pequenas correções de direção. Essa decisão foi tomada porque, em uma versão anterior, a tartaruga tentava andar e virar ao mesmo tempo mesmo estando muito desalinhada, o que gerava loops e curvas grandes.

A velocidade também foi ajustada para deixar o desenho mais rápido. O número de pontos planejados foi reduzido para um valor equilibrado, e o timer do controle passou a enviar comandos com maior frequência. Isso tornou a simulação mais prática de assistir, sem perder completamente os detalhes principais do cachorro.

## Dificuldades encontradas

A primeira dificuldade foi o caminho da imagem. Como o script podia ser executado de diretórios diferentes, o uso de `dog.jpg` como caminho relativo causava erro no carregamento. A solução foi calcular o caminho com base no arquivo Python.

Outra dificuldade foi respeitar o requisito de usar OpenCV apenas para carregar a imagem. Inicialmente havia uso de `cv2.cvtColor`, mas isso foi substituído por uma conversão manual em NumPy.

Também houve problemas na integração com o pacote ROS 2. A imagem `dog.jpg` não era instalada junto com o pacote, então o controller funcionava no código fonte, mas falhava quando executado com `ros2 run`. Para resolver isso, o `setup.py` foi ajustado para incluir o pacote raiz e instalar a imagem como dado do pacote.

A maior dificuldade técnica foi transformar bordas em movimento. A imagem binária parecia correta, mas a tartaruga fazia loops e rabiscos porque os pontos não estavam ordenados como um contorno real. A solução foi separar componentes conectados, criar traços locais e usar quebras de trajeto com a caneta levantada. Isso aproximou melhor o desenho final do contorno visualizado na etapa de visão.

## Conclusão

O projeto conseguiu integrar visão computacional e controle robótico em ROS 2. A pipeline extrai os contornos do cachorro usando implementações próprias de conversão para cinza, convolução, suavização e Sobel. Em seguida, o controller transforma esses contornos em trajetórias e comanda a tartaruga para desenhá-los no `turtlesim`.

Apesar das limitações naturais do `turtlesim`, como a dificuldade de desenhar muitos detalhes pequenos com movimento contínuo, o resultado mostra a estrutura principal do cachorro e demonstra a conexão entre processamento de imagem e ação de um robô simulado. As principais melhorias feitas ao longo do desenvolvimento foram a organização dos pontos do contorno, o uso da caneta para separar traços e o ajuste do controle para evitar loops.