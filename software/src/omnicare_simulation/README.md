# OmniCare Simulation (`omnicare_simulation`)

O **`omnicare_simulation`** é o pacote responsável por toda a **infraestrutura de simulação do OmniCare**, permitindo testar e validar o comportamento do robô em um ambiente virtual antes da execução no robô real.  
Ele integra o OmniCare ao **Gazebo**, fornece *launch files* dedicados e implementa **serviços auxiliares**, como o teletransporte do robô entre posições e andares.

Este pacote é essencial para desenvolvimento, depuração e validação segura das funcionalidades de navegação, controle e comportamento.

---

## Considerações

A simulação foi projetada para:

- Reproduzir o comportamento do robô real com alta fidelidade;
- Permitir testes rápidos e repetíveis;
- Facilitar o desenvolvimento de navegação multi-andares;
- Reduzir riscos durante a fase de desenvolvimento.

Sempre que possível, a mesma lógica utilizada no robô real é reaproveitada na simulação, alterando apenas os *plugins* e interfaces específicas do ambiente virtual.

---

## Arquitetura Geral

O `omnicare_simulation` atua como uma camada intermediária entre:

- A **descrição do robô** (`omnicare_description`);
- O **stack de navegação** (`navigation_pkg`);
- O **Behavior Manager** (`omnicare_behavior`);
- O **simulador Gazebo**.

Ele garante que todos esses módulos operem de forma coordenada dentro do ambiente simulado.

---

## Arquitetura do Pacote

```bash
omnicare_simulation/
│ 
├─ launch/
│  └─ simulation.launch.py        # Launch principal da simulação
│
├─ omnicare_simulation/
│  └─ teleport_service.py         # Serviço de teletransporte no Gazebo 
│
└─ simulation
   ├─ models/ # alguns models para o mundo do Gazebo
   ├─ stl/ # mapas criados em STL, como da FEI e do HU-USP
   └─ words/ # os mundos do Gazebo, tanto da FEI, quanto da USP
```

---

## Funcionalidades Principais

- Inicialização completa do OmniCare em simulação;
- Spawn do robô no Gazebo com sensores simulados;
- Integração com o ROS 2 Control em modo simulado;
- Serviços de **teletransporte**, utilizados para simular troca de andares;
- Suporte a testes de navegação, comportamento e percepção.

---

## Serviço de Teletransporte

O pacote implementa um **serviço de teletransporte** que permite reposicionar o robô diretamente no ambiente do Gazebo.  
Esse recurso é fundamental para a simulação de navegação multi-andares, pois substitui fisicamente o uso de elevadores reais.

O Behavior Manager utiliza esse serviço para:
- Reposicionar o robô após a troca de mapa;
- Garantir coerência entre a pose simulada e o novo andar.

---

## Como Executar a Simulação

Exemplo genérico de execução:

```bash
ros2 launch omnicare_simulation simulation.launch.py
```
Para spawnar o robo no ambiente, deve se executar o bringupd do simulation robot:
```bash
ros2 launch omnicare_simulation simulation.launch.py
```



Após a inicialização, o robô estará disponível no Gazebo e pronto para receber comandos de navegação, teleoperação ou missões automáticas via Behavior Manager.

---

## Integração com Outros Pacotes

O `omnicare_simulation` integra-se diretamente com:

- **omnicare_description** — descrição física e plugins do robô;
- **navigation_pkg** — navegação e mapeamento;
- **omnicare_behavior** — controle de missões e FSM;
- **omnicare_control** — controladores em modo simulado;
- **omnicare_msgs** — troca de mensagens, serviços e actions.

---

## Papel no Sistema

O **`omnicare_simulation`** permite que todo o sistema OmniCare seja desenvolvido e validado de forma segura, controlada e reprodutível.  
Ele é a principal ferramenta para testes de navegação multi-andares, integração entre módulos e avaliação do comportamento do robô antes da execução em ambientes reais.