# Serial Interface Package (`serial_interface_pkg`)

O **`serial_interface_pkg`** é o pacote responsável por implementar uma **interface de comunicação serial direta** entre o OmniCare e o **manipulador frontal**, sem utilizar o pipeline do **ROS 2 Control**.  
Ele foi desenvolvido para atender às necessidades específicas do manipulador, permitindo um controle mais simples, direto e flexível.

Este pacote complementa a arquitetura de controle do OmniCare, coexistindo com o ROS 2 Control utilizado na base móvel.

---

## Considerações

O manipulador do OmniCare possui requisitos diferentes da base omnidirecional, como:

- Movimentos discretos e bem definidos;
- Sequências de ação simples (ex.: estender, recolher, pressionar botão);
- Comunicação direta com o microcontrolador do manipulador.

Por esse motivo, optou-se por uma **interface serial customizada**, desacoplada do ROS 2 Control, reduzindo complexidade e facilitando a integração e depuração.

---

## Arquitetura Geral

O `serial_interface_pkg` atua como uma **ponte direta** entre o ROS 2 e o microcontrolador responsável pelo manipulador.

Fluxo simplificado:

1. Um nó ROS 2 envia um comando para o manipulador;
2. O `serial_interface_pkg` converte esse comando para o protocolo serial definido;
3. O microcontrolador executa a ação solicitada;
4. (Opcional) O microcontrolador retorna um feedback de estado.

Esse fluxo garante comunicação simples e determinística para as ações do manipulador.

---

## Funcionalidades Principais

- Comunicação serial direta com o microcontrolador do manipulador;
- Envio de comandos de acionamento (ex.: estender, recolher, parar);
- Possibilidade de leitura de feedback ou estados;
- Parametrização de porta serial e baudrate;
- Independência do ROS 2 Control.

---

## Integração com Outros Pacotes

O `serial_interface_pkg` é utilizado principalmente por:

- **`omnicare_hri`** — integração com ações de interação humano–robô, quando aplicável no presentation do robo;
- **Firmware do manipulador** — responsável por executar os comandos recebidos via serial, respeitando o sensor de fim de curso.

Esse pacote atua como elo direto entre o comportamento de alto nível e o hardware do manipulador.

---

## Como Executar

Para iniciar a interface serial do manipulador, execute:

```bash
ros2 launch serial_interface_pkg serial_interface.launch.py robot_part:=manip
```

Antes de executar, verifique se:

- A porta serial correta está configurada nos arquivos de configuração (**`/config/*.yaml`**);
- O microcontrolador do manipulador está energizado;
- O firmware do manipulador está em execução.

---

## Papel no Sistema

O **`serial_interface_pkg`** é responsável por garantir que o manipulador do OmniCare possa ser controlado de forma **simples, confiável e independente** da base móvel.  
Ele complementa a arquitetura do sistema ao permitir uma comunicação direta e específica para o manipulador, mantendo o restante do sistema modular e organizado.


## Observação

O USB do Arduino ñ esta linkado com algum nome de USB padrão, então possivelmente possa mudar do **`ttyACM0`**