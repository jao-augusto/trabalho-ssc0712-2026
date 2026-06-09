# capote

Pacote de ROS2 Jazzy em Ubuntu 24.04 para a disciplina SSC0712 - Programação de Robôs Móveis.

O pacote foi feito para o controle de um robô autônomo em um jogo de capturar a bandeira.

Em desenvolvimento...

Os slides ainda estão sendo feitos, desculpe não conseguir entregar agora :/

---

## Instalação

Dentro da pasta `src` do seu workspace ROS 2, clone o repositório:

```bash
cd ~/seu_workspace/src
mkdir capote
cd capote
git clone https://github.com/jao-augusto/trabalho-ssc0712-2026.git
```

Depois siga para a compilação abaixo.

---

## Compilação

```bash
cd ~/seu_workspace
colcon build --symlink-install --packages-select capote
source install/setup.bash
```

---

## Execução

Execute cada comando em um terminal separado já com o source, nessa ordem:

```bash
# 1. Inicia a simulação (Gazebo)
ros2 launch capote inicia_simulacao.launch.py

# 2. Carrega o robô no ambiente
ros2 launch capote carrega_robo.launch.py

# 3. Inicia o nó de controle
ros2 run capote controle_robo
```

---

## Arquitetura do sistema

O nó principal `controle_robo` integra LiDAR, odometria e câmera para navegar autonomamente até uma bandeira e se posicionar para capturá-la, usando uma máquina de estados:

- **vagando** — explora o ambiente desviando de obstáculos até avistar a bandeira
- **bandeira_avistada** — para 3 segundos para confirmar que é a bandeira
- **navegando** — vai em direção à bandeira pelo ângulo gravado; se perder de vista, gira para o lado em que ela estava
- **chegou** — alinha pela câmera, refina pelo LiDAR e avança até a posição de pega
