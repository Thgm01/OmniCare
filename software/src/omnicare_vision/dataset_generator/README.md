# OmniCare Dataset Generator (`dataset_generator`)

O **`dataset_generator`** é o pacote responsável por **gerar automaticamente datasets de visão computacional** a partir de dados gravados durante a operação do OmniCare, principalmente **rosbags** contendo imagens da câmera do robô.  
Ele foi desenvolvido para apoiar o treinamento e a melhoria contínua dos modelos utilizados no pacote **`floor_detector`**.

---

## Objetivo do Pacote

Durante testes reais ou simulados, o robô grava rosbags contendo imagens da câmera frontal.  
O `dataset_generator` permite transformar esses dados brutos em **datasets organizados**, prontos para:

- Anotação manual (ex.: Roboflow);
- Treinamento de modelos de detecção ou classificação;
- Avaliação e validação de novos modelos.

Esse processo reduz drasticamente o esforço manual de coleta de dados.

---

## Arquitetura Geral

O pacote funciona de forma **offline**, ou seja, não participa da execução do robô em tempo real.  
Seu fluxo principal é:

1. Leitura de uma rosbag contendo imagens;
2. Extração dos frames da câmera;
3. Salvamento das imagens em diretórios organizados;
4. Geração de estrutura compatível com ferramentas de anotação.

---

## Arquitetura do Pacote

```bash
dataset_generator/
├─ config/
│  └─ dataset_generator_config.yaml      # Configurações do processo de geração
│
└─ dataset_generator/
   ├─ dataset_generator.py               # Script principal de geração do dataset
   └─ get_parameters.py                  # Funções auxiliares 

```

---

## Funcionamento

O usuário fornece como entrada:

- O caminho para a rosbag;
- O tópico de imagem desejado (ex.: `camera1/image_raw`);
- Parâmetros como taxa de amostragem e diretório de saída.

O pacote então percorre a rosbag e salva os frames em formato de imagem, organizados de forma consistente.

---

## Exemplo de Uso

Exemplo genérico de execução:

```bash
ros2 launch dataset_generator dataset_generator.launch.py
```

> Os argumentos exatos podem variar conforme a implementação do script principal.

---

## Integração com Outros Pacotes

O `dataset_generator` é utilizado principalmente em conjunto com:

- **floor_detector** — geração de dados para treinamento dos modelos;
- **omnicare_vision / usb_cam** — origem das imagens;
- Ferramentas externas de anotação (ex.: Roboflow).

---

## Papel no Sistema

O **`dataset_generator`** viabiliza a criação de datasets representativos do ambiente real do robô, permitindo a evolução contínua dos modelos de visão do OmniCare.  
Ele é um componente fundamental para garantir robustez e generalização dos algoritmos de percepção utilizados na navegação multi-andares.