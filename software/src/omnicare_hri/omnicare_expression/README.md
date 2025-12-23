# OmniCare Expression (`omnicare_expression`)

O **`omnicare_expression`** é o pacote responsável pela **expressividade e sinalização do OmniCare**, utilizando uma tela frontal com animações (rosto do robô) e LEDs/Buzzer para tornar a presença do robô mais amigável e comunicativa.  
O pacote permite executar vídeos/animações de forma sincronizada com ações do robô, ajudando a transmitir estados e intenções durante demonstrações e missões.

---

## Considerações

As animações exibidas no OmniCare são **pré-renderizadas** (vídeos gerados previamente) e reproduzidas localmente na unidade de processamento do robô.  
O sistema foi implementado com foco em:

- Baixa latência e reprodução estável;
- Sincronização de ações do robô com timestamps do vídeo;
- Uso em testes e demonstrações públicas.

Embora contribua para aceitação social do robô, este pacote não caracteriza HRI bidirecional, atuando como uma camada de **expressão visual**, apenas para apresentação do projeto em ocasiões específicas.

---

## Arquitetura Geral

O pacote reproduz um vídeo principal e chama serviços sincronizadas [(**`expressionsNode.py`**)](omnicare_expression/expressionService.py) conforme o tempo do vídeo, como:

- Movimentações da base para demonstração da cinemática omnidirecional;
- Acionamento do manipulador;
- LEDs e buzzer.

A sincronização ocorre por **marcadores temporais** definidos no próprio código [(**`expressionsNode.py`**)](omnicare_expression/expressionsNode.py).

O pacote também faz a interface com o microcontrolador responsável pelos LEDs e Buzzer  [(**`alertNode.py`**) ](omnicare_expression/alertNode.py)

---

## Componentes do Pacote

- Nó principal de reprodução de animações;
- Vídeos e assets gráficos;
- Arquivo de configuração com labels/timestamps;
- Integração opcional com serviços do robô.

---


## Como Executar

Para inicializar o rosto amigável e a interface com o microcontrolador deve-se rodar esse launch:

```bash
ros2 launch omnicare_expression expression_system.launch.py
```

Quando iniciado, o vídeo inicia em estado de idle, aguardando o tópico **`/omnicare/hri/idle_done`**, portanto deve-se chama-lo via CLI e aguardar alguns segundos para começar a apresentação.

```bash
ros2 topic pub -1 /omnicare/hri/idle_done std_msgs/msg/String "{data: True}"
```

---

## Papel no Sistema

O **`omnicare_expression`** fornece ao OmniCare uma camada de **presença social e comunicação visual**, permitindo apresentações, sinalização de estados e sincronização de ações, tornando o robô mais compreensível e amigável.