# OmniCare Description (`omnicare_description`)

O **`omnicare_description`** é o pacote responsável por concentrar toda a **descrição física e estrutural do OmniCare**, servindo como a base para **visualização, simulação e controle** do robô.  
Ele define como o robô “existe” dentro do ecossistema ROS 2, garantindo consistência entre o modelo físico, o simulador e os controladores.

---

## Considerações

A descrição do robô foi projetada para atender simultaneamente aos seguintes cenários:

- Visualização no **RViz**;
- Simulação física no **Gazebo**;
- Integração com o **ROS 2 Control**;
- Operação no robô real.

Para isso, o pacote foi estruturado de forma modular, permitindo ativar ou desativar plugins conforme o modo de execução (simulação ou real), sem a necessidade de manter múltiplas descrições distintas.

---

## Arquitetura Geral

O `omnicare_description` atua como o **núcleo descritivo** do sistema, fornecendo:

- Estrutura mecânica do robô;
- Relações cinemáticas entre links e juntas;
- Definição de sensores e atuadores;
- Interfaces necessárias para controle e simulação.

Todos os outros pacotes — navegação, controle, simulação e bringup — dependem diretamente dessa descrição. Ele segue essa estrutura de arquivos:
```bash
omnicare_description/
├─ launch/
│  └─ display.launch.py
│     └─ Launch para visualização do robô no RViz
│        (robot_state_publisher + joint_state_publisher + RViz)
│
├─ config/
│  └─ display.rviz
│     └─ Configuração padrão do RViz utilizada no display.launch.py
│
├─ urdf/
│  ├─ robot.xacro
│  │  └─ Arquivo principal da descrição do OmniCare
│  │     Define links, joints e inclui módulos conforme o modo
│  │     (simulação ou robô real)
│  │
│  ├─ robot.gazebo
│  │  └─ Plugins para simulação no Gazebo
│  │     (sensores e parâmetros físicos)
│  │
│  ├─ ros2_control.xacro
│  │  └─ Integração do robô real com o ROS 2 Control
│  │     Define hardware interface, joints e interfaces de comando
│  │
│  ├─ gazebo_control.xacro
│  │  └─ Integração do robô simulado com o ROS 2 Control
│  │     Utiliza o plugin gazebo_ros2_control (ñ utilizado)
│  │
│  └─ camera.xacro
│     └─ Definição dos frames e da geometria da câmera do robô
│
└─ meshes/
   └─ (Arquivos de malha e sensores)
      └─ Modelos 3D utilizados pelo URDF para visualização e colisão
```

---

## URDF / Xacro

A descrição do OmniCare é construída utilizando **URDF com Xacro**, o que permite:

- Parametrização de componentes;
- Reutilização de macros;
- Diferenciação entre **modo simulado** e **modo real**;
- Organização clara do modelo do robô.

O arquivo Xacro principal é responsável por incluir:
- Base omnidirecional;
- Rodas e juntas;
- Manipulador frontal;
- Sensores (ex.: LiDAR);
- Interfaces de controle.

---

## Integração com ROS 2 Control

A integração com o ROS 2 Control é definida diretamente no URDF por meio da tag `<ros2_control>`.  
Essa seção especifica como o sistema de controle se comunica com o hardware real, quais juntas são controladas e quais interfaces de comando e estado são utilizadas.

---

## Definição do Sistema ROS 2 Control

```xml
<ros2_control name="RealRobot" type="system">
```

- **`name="RealRobot"`**: identifica o sistema de controle do robô real.
- **`type="system"`**: indica que o plugin implementa leitura de estados e escrita de comandos diretamente no hardware.

---

## Hardware Interface

```xml
<hardware>
    <plugin>omnidrive_stm32/OmniDriveSTM32</plugin>
```

O plugin **`OmniDriveSTM32`** (encontrado no **`omnicare_control/omnidrive_stm32/`**) implementa a Hardware Interface do ROS 2 Control, sendo responsável por:

- Enviar comandos de velocidade ao STM32;
- Ler feedback dos encoders;
- Atualizar os estados das juntas no ROS 2.

---

## Parâmetros de Hardware

```xml
<param name="left_wheel_name">wheel2_speed</param>
<param name="right_wheel_name">wheel1_speed</param>
<param name="back_wheel_name">wheel3_speed</param>
```

Mapeamento entre juntas do URDF e rodas físicas do robô, essa informação tem que bater com o arquivo de configuração do controlador ([omnidirectional_controllers](../omnicare_control/omnidirectional_controllers/config/omnidirectional_controller.yaml)).

```xml
<param name="loop_rate">30</param>
```

Frequência de atualização da interface (Hz).

```xml
<param name="device">/dev/usb-user</param>
<param name="baud_rate">115200</param>
<param name="timeout_ms">1000</param>
```

Parâmetros da comunicação serial com o STM32.

```xml
<param name="enc_counts_per_rev">12770</param>
```

Resolução do encoder utilizada para cálculo de velocidade e posição (relacionado ao nosso motor - se trocar, vai precisar alterar esse parâmetro).

---

## Definição das Juntas

Cada roda é definida como uma junta controlada:

```xml
<joint name="wheel1_speed">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
</joint>
```

- **Command Interface**: `velocity` (utilizando na função de **write** do [Hardware Inteface](../omnicare_control/omnidrive_stm32/hardware/diffbot_system.cpp))
- **State Interfaces**: `position` e `velocity` (utilizando na função de **read** do [Hardware Inteface](../omnicare_control/omnidrive_stm32/hardware/diffbot_system.cpp))

O mesmo padrão é utilizado para as três rodas omnidirecionais.

---

## Fluxo de Funcionamento

1. Nav2, teleoperação ou Behavior Manager publicam `cmd_vel`;
2. O `controller_manager` recebe o comando;
3. O controlador omnidirecional calcula:
   - cinemática;
   - odometria;
   - velocidades individuais das rodas;
4. O ROS 2 Control envia comandos para o STM32;
5. O STM32 executa o controle de baixo nível (PID/PWM);
6. O feedback dos encoders retorna ao ROS 2;
7. Estados e odometria alimentam Nav2 e localização.

---
## Integração com Gazebo

Para simulação, o `omnicare_description` inclui **plugins do Gazebo** responsáveis por:

- Simular sensores (LiDAR, estados das juntas, etc.);
- Aplicar dinâmica e física ao modelo;
- Permitir interação com o ambiente virtual.

Esses plugins são habilitados apenas quando o robô é executado em modo simulado, mantendo a descrição compatível com o robô real.

---

## Visualização no RViz

A descrição fornecida por este pacote é utilizada diretamente pelo **RViz**, permitindo:

- Visualização da geometria do robô;
- Acompanhamento da árvore de TFs;
- Monitoramento do estado das juntas;
- Depuração visual durante desenvolvimento e testes.

Para conseguir visualiza-lo basta apenas rodar:
```bash
ros2 launch omnicare_description display.launch.py
```

---

## Integração com Outros Pacotes

O `omnicare_description` é utilizado diretamente por:

- **`omnicare_bringup`** — carregamento da descrição durante a inicialização;
- **`omnicare_control`** — definição das juntas e transmissões para controle;
- **`omnicare_simulation`** — spawn do robô no Gazebo;
- **`omnicare_navigation`** — referência de frames (`base_link`, `odom`, `map`);
s
---

## Papel no Sistema

O **`omnicare_description`** é o pacote que garante que **todas as camadas do OmniCare falem a mesma “língua física”**.  
Ele assegura que o robô visto no simulador, controlado pelo ROS 2 Control e operando no mundo real compartilhe a mesma estrutura, cinemática e referências, sendo essencial para a robustez e a escalabilidade do sistema.
