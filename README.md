# OmniCare: Robô modularizado omnidirecional para navegação em vários andares via elevador


Este repositório reúne os arquivos de desenvolvimento do projeto **OmniCare**, um robô móvel omnidirecional modularizado, voltado para aplicação em ambientes hospitalares com a capacidade de deslocamento autônomo entre andares utilizando elevadores. O projeto integra o Trabalho de Conclusão de Curso (TCC) em Engenharia de Robôs pela FEI.

## 📄 Sobre o Projeto

Desenvolveu-se com base em uma **estrutura modular omnidirecional**, contando com:
- Um **manipulador cartesiano** para acionamento de botões (como os de elevadores);
- Uma **arquitetura elétrica integrada** com sensores, atuadores e controle embarcado;
- Sistema de controle e navegação autônoma com uso de pacotes do **ROS 2 Humble (Slam Toolbox e NAV2)**.

## 📁 Estrutura do Repositório

```bash
.
├── hardware/ # Documentação e arquivos da parte eletrônica
├── mechanics/ # Arquivos CAD e modelagem mecânica
├── software/src/ # Fonte do ROS 2 workspace
│   ├── omnicare_bringup/ # Launch principal do OmniCare
│   ├── omnicare_control/ # Controle de movimento do robô
│   ├── omnicare_description/ # Descrição do robô (URDF/XACRO)
│   ├── omnicare_msgs/ # Definição de mensagens customizadas
│   ├── omnicare_navigation/ # Configuração do SLAM e NAV2
│   ├── omnicare_simulation/ # Simulação no Gazebo
│   └── omnicare_vision/ # Módulo de visão computacional
├── scripts.sh # Script para inicialização e setup
├── setup.sh # Configurações de ambiente (ex: regras do RPLiDAR)
└── README.md # Este documento
```
## 👤 Autores

**Lucas Lagoeiro**  
Graduando em Engenharia de Robôs – Centro Universitário FEI  
Área: Programação e Elétrica   
[LinkedIn](https://www.linkedin.com/in/llagoeiro/) • [GitHub](https://github.com/LucasLagoeiro)

**Thiago Moura**  
Graduando em Engenharia de Automação e Controle – Centro Universitário FEI  
Área: Programação e Elétrica  
[LinkedIn](https://www.linkedin.com/in/thiago-t-moura/) • [GitHub](https://github.com/Thgm01)

**Massiel Blandy**  
Graduando em Engenharia de Automação e Controle – Centro Universitário FEI  
Área: Programação  
[LinkedIn](https://www.linkedin.com/in/massiel-blandy-ram%C3%B3n-65214829a/) • [GitHub](https://github.com/massiblandy)

**Júlia Hernandes**  
Graduanda em Engenharia Elétrica – Centro Universitário FEI  
Área: Elétrica  
[LinkedIn](https://www.linkedin.com/in/j%C3%BAlia-gagliera-hernandes-40545b221/) 

**Gustavo Rosell**  
Graduando em Engenharia de Automação e Controle – Centro Universitário FEI  
Área: Mecânica  
[LinkedIn](https://linkedin.com/in/gustavo-rosell) 

**Leonardo Quirino**  
Graduando em Engenharia de Robôs – Centro Universitário FEI  
Área: Mecânica  
[LinkedIn](https://www.linkedin.com/in/leonardo-quirino-353486218/)


