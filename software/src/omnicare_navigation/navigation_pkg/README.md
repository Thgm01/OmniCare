# OmniCare Navigation (`navigation_pkg`)

O **`navigation_pkg`** é o pacote responsável por toda a **infraestrutura de navegação autônoma do OmniCare**.  
Ele concentra os *launch files* do Nav2, configurações de mapeamento e localização, mapas dos ambientes, definição de checkpoints e nós auxiliares que viabilizam a navegação **multi-andares** integrada ao Behavior Manager.

Este pacote é um dos núcleos do sistema, pois conecta percepção, controle e comportamento em uma navegação robusta e reprodutível.

---

## Considerações

A navegação do OmniCare foi projetada para:

- Operar em ambientes reais e simulados;
- Suportar troca dinâmica de mapas;
- Executar trajetórias baseadas em checkpoints pré-definidos;
- Integrar-se ao Nav2, SLAM Toolbox, robot_localization e AMCL;
- Permitir extensões para novos ambientes e andares sem alterar código-fonte.

---

## Arquitetura do Pacote

```bash
navigation_pkg/
├─ launch/
│  ├─ navigation2.launch.py
│  │  └─ Launch principal do Nav2, responsável por subir o stack de navegação
│  │     (planners, controllers, costmaps, BT e recovery behaviors)
│  │
│  ├─ mapping_slam_toolbox.launch.py
│  │  └─ Launch para mapeamento utilizando SLAM Toolbox
│  │
│  └─ mapping_cartographer.launch.py
│     └─ Launch alternativo para mapeamento com Cartographer (ñ utilizado)
│
├─ config/
│  ├─ nav/
│  │  ├─ nav2_params.yaml
│  │  │  └─ Parâmetros principais do Nav2 (planners, controllers, costmaps, BT)
│  │  │
│  │  ├─ nav2_params_MPPI.yaml
│  │  │  └─ Variante de configuração utilizando o controlador MPPI
│  │  │
│  │  ├─ sim_nav2_params.yaml
│  │  │  └─ Parâmetros específicos para navegação em simulação
│  │  │
│  │  ├─ ekf.yaml
│  │  │  └─ Configuração do robot_localization (EKF) - obs: é utilizando o pacote
│  │  │     do robot localization no bringup do robo real, por isso desse config
│  │  │
│  │  ├─ slam_toolbox.yaml
│  │  │  └─ Parâmetros do SLAM Toolbox
│  │  │
│  │  └─ box_filter.yaml
│  │     └─ Filtros aplicados ao LiDAR para aumentar robustez da navegação
│  │
│  ├─ maps/
│  │  ├─ FEI/
│  │  │  └─ Mapas do ambiente FEI
│  │  │
│  │  ├─ HU_USP/
│  │  │  └─ Mapas do ambiente HU-USP
│  │  │
│  │  └─ simulation/
│  │     └─ Mapas utilizados em simulação
│  │
│  └─ checkpoint/
│     └─ Arquivos JSON contendo sequências de checkpoints (poses)
│        organizadas por ambiente e andar
│
└─ navigation_pkg/
   ├─ checkpointsService.py
   │  └─ Serviço responsável por dar start, parar e salvar checkpoints ao sistema
   │
   ├─ floorManagerNode.py
   │  └─ Nó de gerenciamento de contexto de andar e mapas (utilizado apenas para depuração)
   │
   ├─ switchFloorService.py
   │  └─ Serviço para troca de andar/mapa durante a missão (Behavior Manager)
   │
   ├─ enterElevatorAction.py
   │  └─ Action relacionada à navegação associada ao uso do elevador (Foi desenvolvida, 
   │     mas ñ utilizada no Behavior Manager)
   │
   ├─ poseFromTFNode.py
   │  └─ Nó auxiliar para obtenção da pose a partir da árvore TF (URDF)
   │
   └─ amclOrientationNode.py
      └─ Nó auxiliar para ajuste e consistência da orientação do AMCL
```

---

## Funcionamento Geral

1. O Nav2 é inicializado;
2. O mapa correspondente é carregado;
3. Os checkpoints são lidos;
4. O robô navega sequencialmente entre checkpoints;
5. Em missões multi-andares, ocorre troca dinâmica de mapa;
6. A navegação é retomada até o destino final.

---

## Integração com Outros Pacotes

- omnicare_behavior  
- omnicare_control  
- omnicare_description  
- omnicare_vision  
- omnicare_msgs  

---

## Papel no Sistema

O **`navigation_pkg`** fornece serviços e actions para o alto nível (behavior_manager), viabilizando a navegação segura e a troca dinâmica de mapas em ambientes multi-andares.