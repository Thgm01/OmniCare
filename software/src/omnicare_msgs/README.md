# OmniCare Messages (`omnicare_msgs`)

O **`omnicare_msgs`** é o pacote responsável por centralizar todas as **mensagens customizadas** utilizadas no projeto OmniCare.  
Ele define os tipos de dados usados para comunicação entre nós ROS 2 por meio de **topics**, **services** e **actions**, garantindo padronização, clareza e integração entre os diferentes subsistemas do robô.

---

## Considerações

Em um sistema robótico complexo como o OmniCare, a comunicação entre módulos precisa ser:

- Bem definida;
- Consistente ao longo de todo o projeto;
- Fácil de manter e expandir.

Por esse motivo, todas as mensagens customizadas foram agrupadas em um único pacote, evitando duplicações e facilitando a evolução do sistema.

---

## Tipos de Interfaces

O `omnicare_msgs` contém três tipos principais de definições:

### Messages (`msg`)
Mensagens utilizadas para comunicação assíncrona via tópicos, geralmente para:
- Estados do robô;
- Comandos simples;
- Publicação contínua de informações.

---

### Services (`srv`)
Serviços utilizados para chamadas síncronas, quando é necessário:
- Enviar uma requisição;
- Aguardar uma resposta imediata.

São comuns em comandos pontuais, como inicializações ou mudanças de estado.

A maioria dos serviços criados são utilizadas mensagens customizadas para aumentar a flexibilidade do que mandar.

---

### Actions (`action`)
Actions são utilizadas para operações de **longa duração**, permitindo:
- Envio de um objetivo (*goal*);
- Recebimento de *feedback* contínuo;
- Notificação de conclusão (*result*).

No OmniCare, actions são fundamentais para tarefas como:
- Navegação multi-andares;

---

## Integração com Outros Pacotes

Este pacote é utilizado por praticamente todo o sistema, incluindo:

- **`omnicare_behavior`** — definição de actions para missões e estados;
- **`omnicare_navigation`** — troca de informações de navegação;
- **`omnicare_control`** — comandos e estados de controle;
- **`omnicare_expression`** — acionamento de expressões e sinalizações;
- **`omnicare_hri`** — interação humano–robô.

---

## Papel no Sistema

O **`omnicare_msgs`** garante que todos os módulos do OmniCare se comuniquem de forma clara, consistente e organizada.  
Ele é um elemento fundamental para a escalabilidade do projeto, permitindo adicionar novas funcionalidades sem comprometer a integração entre os subsistemas existentes.