# Navegação

- [Configuração do ambiente virtual para rodar a atividade](#configuração-do-ambiente-virtual-para-rodar-a-atividade)
- [Configuração do pacote python](#configuração-do-pacote-python)
- [Rodando meu pacote em seu computador](#rodando-meu-pacote-em-seu-computador)
- [Explicação do código e suas principais funções](#explicação-do-código-e-suas-principais-funções)
  - [Método de Navegação Reativa](#método-de-navegação-reativa)
  - [Método de Navegação com Mapa](#método-de-navegação-com-mapa)
  - [Função Inicializadora](#função-inicializadora)
- [Vídeo que comprova plenamente o funcionamento do sistema criado](#vídeo-que-comprova-plenamente-o-funcionamento-do-sistema-criado)

# Atividade ponderada - Navegação no Labirinto

&emsp;**Objetivo:** Implementar dois métodos distintos de navegação - **Navegação Reativa** e **Navegação com Mapa** - para um robô autônomo. O robô deve navegar até o alvo representado no mapa enquanto utiliza diferentes estratégias de movimento.

#### Aviso:

&emsp;Parte do código desenvolvido nesta atividade foi reaproveitado de implementações colaborativas feitas com colegas.

---

# Configuração do ambiente virtual para rodar a atividade:

&emsp;Primeiro, clone o repositório com o seguinte comando:
```
git clone git@github.com:gustavoesteves0/Ponderadas-M08-EC.git
```
&emsp;Depois de clonar o repositório, navegue até a pasta raiz do projeto no terminal, ative o ambiente virtual do Python com os comandos abaixo e instale as dependências necessárias:

```
python3 -m venv venv

source venv/bin/activate

python3 -m pip install -r requirements.txt
```

# Configuração do pacote python:

&emsp;Para configurar o pacote e integrá-lo ao ROS2, navegue até o diretório raiz do projeto e execute os comandos abaixo:
```bash
colcon build

source install/local_setup.bash
```
# Rodando meu pacote em seu computador:

1. Inicie o simulador do labirinto com o comando:
```bash
ros2 run cg maze
```
2. Após o simulador ser carregado, você verá o robô azul (ponto de partida) e o alvo vermelho no mapa.

3. Escolha no terminal o método de navegação que deseja utilizar:
   - Navegação Reativa
   - Navegação com Mapa

4. O robô começará a navegar no labirinto. O progresso será exibido no terminal.

---

# Explicação do código e suas principais funções:

## Método de Navegação Reativa

&emsp;O método de navegação reativa utiliza exclusivamente informações obtidas em tempo real dos sensores para navegar no labirinto. Sem acesso ao mapa completo, o robô toma decisões baseadas no ambiente ao seu redor.

- **Fluxo**:
  1. O robô verifica os sensores para identificar os estados dos quadrados adjacentes (livre, bloqueado ou alvo).
  2. Toma decisões com base na menor distância até o alvo.
  3. Utiliza estratégias de contorno de obstáculos caso encontre barreiras.

### Funções Principais:

1. `send_move_command`: Envia comandos de movimento para o robô e retorna a resposta do serviço.
2. `follow_the_target`: Calcula o movimento ideal para aproximar o robô do alvo.
3. `wall_following`: Estratégia auxiliar para contornar obstáculos quando o caminho direto está bloqueado.
4. `navigate`: Gerencia o fluxo principal de navegação, integrando as estratégias mencionadas.

---

## Método de Navegação com Mapa

&emsp;O método de navegação com mapa planeja a rota do robô utilizando o algoritmo A* com base no mapa completo do labirinto.

- **Fluxo**:
  1. O mapa é obtido através do serviço `/get_map`.
  2. O algoritmo A* planeja a rota até o alvo utilizando a heurística de distância Manhattan.
  3. O robô segue o caminho planejado, ajustando-se caso encontre problemas durante a execução.

### Funções Principais:

1. `get_map`: Obtém o mapa completo do labirinto do serviço ROS.
2. `plan_path`: Implementa o algoritmo A* para calcular a rota otimizada.
3. `navigate`: Executa a navegação seguindo a rota planejada.

---

## Função Inicializadora

&emsp;A função principal permite que o usuário escolha entre os métodos de navegação e gerencia a execução do simulador e do sistema de navegação.

### Estrutura:

1. A interface CLI (Command Line Interface) oferece as opções de navegação.
2. O programa gerencia o simulador e o sistema ROS de forma simultânea utilizando threads.
3. Logs detalhados são exibidos no terminal, informando o progresso do robô.
