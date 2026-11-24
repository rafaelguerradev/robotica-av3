# Atividade Prática: Construção e Simulação de um Robô Móvel para Monitoramento de Vagas de Estacionamento

Com o crescimento dos centros urbanos, a busca por vagas em estacionamentos tornou-se uma tarefa demorada e ineficiente. Sistemas inteligentes de monitoramento podem auxiliar motoristas informando a disponibilidade de vagas em tempo real.

Nesta atividade, os alunos deverão projetar um robô móvel autônomo responsável por realizar inspeções periódicas em um estacionamento, detectando a ocupação das vagas e atualizando um sistema central de monitoramento.

---

## Objetivo

Em uma equipe composta de até cinco pessoas, os alunos deverão construir e simular um robô móvel autônomo capaz de percorrer um estacionamento, identificar vagas ocupadas e desocupadas utilizando **ROS2**.

Ao final da atividade, a equipe deverá demonstrar conhecimentos sobre:
- Modelagem de robôs móveis.
- Simulação em ambiente virtual.
- Arquitetura de nós em ROS2.
- Publicação e assinatura de tópicos.
- Sensoriamento virtual.
- Navegação autônoma.
- Integração entre percepção e tomada de decisão.

---

## Cenário Proposto

O ambiente simulado deverá conter:
- Um estacionamento com pelo menos 5 vagas.
- Veículos posicionados em algumas vagas.
- Corredores para deslocamento do robô.
- Estações de início e retorno.

---

## Requisitos do Robô

O robô deverá possuir:
- Modelo criado em **URDF** ou **Xacro**.
- **Sensores** *(escolher pelo menos um dos seguintes)*:
  - LiDAR 2D.
  - Câmera RGB.
  - Sensor de ultrassom.
- **Atuadores**:
  - Controle de velocidade linear.
  - Controle de velocidade angular.

---

## Tarefas a Desenvolver

### Parte 1 – Modelagem do Robô
Criar o modelo do robô contendo:
- Chassi.
- Rodas.
- Sensores.
- Frames TF necessários.

**Ferramentas de simulação e controle:**
- ROS2 Humble ou Jazzy
- Gazebo
- RViz2
- URDF/Xacro
- Python ou C++

### Parte 2 – Construção do Ambiente
Criar um mundo no Gazebo contendo:
- Estacionamento.
- Vagas delimitadas.
- Veículos simulados.
- Área de circulação.

### Parte 3 – Navegação
Implementar um nó capaz de:
- Percorrer todas as linhas do estacionamento.
- Visitar os pontos de inspeção.
- Retornar ao ponto inicial.

### Parte 4 – Detecção de Vagas
Implementar um algoritmo para classificar cada vaga como:
- **Livre**
- **Ocupada**

A detecção pode ser realizada através de:
- **Opção A – LiDAR:**
  - Verificar presença de obstáculos dentro da região de cada vaga.
- **Opção B – Visão Computacional:**
  - Detectar veículos utilizando OpenCV.
- **Opção C – Sensor Ultrassônico:**
  - Analisar distância média dentro da área da vaga.

---

## Entregas e Prazos

Cada equipe deverá realizar uma apresentação de **20 min.** da simulação e do protótipo no dia **03/12/26**.