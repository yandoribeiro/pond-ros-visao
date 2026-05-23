# Visão Geral

Este repositório documenta o desenvolvimento de uma pipeline completa que conecta visão computacional e controle robótico no ROS2. O objetivo do projeto foi ler uma imagem, extrair seu contorno com algoritmos implementados manualmente e transformar esse contorno em uma trajetória executável no turtlesim.

A construção foi organizada em duas frentes principais. Na etapa de visão, foi criado o notebook pipeline_visao.ipynb, responsável por todo o processamento da imagem: carregamento (OpenCV apenas para leitura), conversão para escala de cinza, suavização gaussiana, detecção de bordas por Sobel manual, binarização, extração de componentes conectados e rastreamento de contornos. O notebook contém explicações técnicas acima dos blocos de código, justificando parâmetros e decisões adotadas ao longo da pipeline.

Na etapa de trajetória e integração com o simulador, foi desenvolvido o notebook pipeline_turtlesim.ipynb. Nele, os contornos exportados pela etapa de visão são carregados, filtrados e mapeados para o espaço de coordenadas do turtlesim (0 a 11), com preservação de proporção e centralização da figura. A saída dessa etapa é um arquivo de waypoints em JSON utilizado pelo nó ROS 2.

A execução no ROS 2 está no pacote ros2_package/src/turtle_contour_follower, com o nó follow_waypoints_node.py, que lê os waypoints e comanda a tartaruga para desenhar o contorno no turtlesim. A pasta results/ armazena os artefatos intermediários e finais da pipeline (imagens geradas, máscaras e arquivos .npy/.npz/.json), servindo como ponte entre os notebooks e o pacote ROS.

Estrutura principal do projeto:

- pipeline_visao.ipynb: processamento de imagem e extração de contornos.

- pipeline_turtlesim.ipynb: mapeamento para turtlesim e geração de waypoints.

- results/: saídas da pipeline (artefatos visuais e dados exportados).

- images/: imagem de entrada utilizada no projeto.

- ros2_package/src/turtle_contour_follower/: pacote ROS 2 com nó de execução da trajetória.

# Passo a passo de execução

1. Executar pipeline de visão

- Abra pipeline_visao.ipynb.

- Execute todas as células em ordem.

- Ao final, confirme que os artefatos foram gerados em results/ (especialmente visao_contours_tracing.npz e arquivos .npy).

2. Executar pipeline de mapeamento para turtlesim

- Abra pipeline_turtlesim.ipynb.

- Execute as células em ordem para carregar os contornos e gerar a trajetória no espaço 0–11.

- Ao final, confirme a geração de results/turtlesim_waypoints.json.


3. Compilar o pacote ROS 2

- No terminal (Ubuntu/WSL), entre na pasta do pacote:

```
cd ros2_package
```

- Carregue o ambiente ROS 2:

```
source /opt/ros/jazzy/setup.bash
```

- Compile:

```
colcon build
```

- Carregue o workspace compilado:

```
source install/setup.bash
```

4. Executar o turtlesim

- Em um terminal com ambiente ROS carregado:

```
ros2 run turtlesim turtlesim_node
```

5. Executar o nó seguidor de waypoints

- Em outro terminal (também com ambiente ROS e workspace carregados):

```
cd ros2_package
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtle_contour_follower follow_waypoints_node --ros-args -p waypoints_file:=/caminho/absoluto/para/pond-ros-visao/results/turtlesim_waypoints.json
```

6. Verificar resultado

- A tartaruga deve percorrer os pontos e desenhar o contorno processado.

Vídeo de demonstração: https://youtu.be/bMkQyrlC4HU