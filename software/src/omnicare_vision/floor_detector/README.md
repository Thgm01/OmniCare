# OmniCare Floor Detector (`floor_detector`)

O **`floor_detector`** é o pacote responsável pelo **reconhecimento automático do andar** em que o OmniCare se encontra, por meio de **visão computacional aplicada ao display do elevador**.  
Esse pacote é um componente-chave para a navegação **multi-andares**, fornecendo ao sistema a informação necessária para validar a troca de andar e sincronizar a mudança de mapas.

---

## Considerações

O reconhecimento do andar é um problema crítico em ambientes reais, onde informações como sensores de posição vertical nem sempre estão disponíveis ou são confiáveis.  
Por isso, o OmniCare utiliza uma abordagem baseada em **visão**, diretamente alinhada com a percepção do ambiente humano.

O pacote foi projetado para:
- Operar em ambientes reais e simulados;
- Ser robusto a variações de iluminação e ângulo;
- Reduzir falsos positivos na detecção do andar;
- Minimizar consumo computacional quando não está ativo.

---

## Arquitetura Geral

O `floor_detector` implementa um pipeline de visão em **duas etapas**:

1. **Detecção do display do elevador** na imagem completa da câmera;
2. **Classificação do dígito do andar** a partir do recorte do display.

Essa arquitetura reduz custo computacional e aumenta a confiabilidade da classificação.

---

## Arquitetura do Pacote

```bash
floor_detector/
├─ launch/
│  └─ floor_detector.launch.py        # Launch do sistema de detecção
│
├─ config/
│  └─ floor_detector_config.yaml      # Parâmetros gerais do detector (como os pesos da rede treinada do HU e da FEI)
│
├─ weights/                           # Se concentra todos os pesos treinados
│
└─ floor_detector/
   ├─ floor_detector_node.py          # Nó principal que faz a detecção e classificação dos andares
   └─ get_parameters.py               # Pega os parametros do config
```

---

## Pipeline de Visão

### 1️⃣ Detecção do Display

O primeiro modelo identifica a **região do display do elevador** na imagem da câmera frontal do robô.  
Essa etapa retorna uma *bounding box* que delimita a área de interesse.

---

### 2️⃣ Classificação do Andar

A região recortada do display é então utilizada como entrada para o segundo modelo, responsável por **classificar o dígito do andar**.  
O resultado é publicado como uma estimativa do andar atual no tópico: **`omnicare/floor_detector/info`**

---

## Estratégia de Robustez

Para reduzir erros de classificação, o sistema:
- Executa múltiplas inferências consecutivas;
- Valida a consistência do resultado ao longo do tempo;
- Apenas confirma o andar quando um número mínimo de classificações iguais é atingido.

Essa lógica é utilizada pelo Behavior Manager para decidir quando a troca de mapa pode ser realizada com segurança.

---

## Como Executar

Para iniciar o detector de andar, execute:

```bash
ros2 launch floor_detector floor_detector.launch.py
```

Porém a inferencia fica desativada para aumentar a autonomia da bateria. Para ativa-lá, basta apenas chamar o serviço desse nó:

```bash
ros2 service call /omnicare/floor_detector/OnOffNode omnicare_msgs/srv/OnOffNode "{activate: True}"
```

---

## Integração com Outros Pacotes

O `floor_detector` integra-se diretamente com:

- **omnicare_behavior** — validação da troca de andar;
- **navigation_pkg** — sincronização da mudança de mapa;
- **omnicare_vision / usb_cam** — aquisição de imagens da câmera;
- **omnicare_msgs** — troca de mensagens customizadas.

---

## Papel no Sistema

O **`floor_detector`** fornece ao OmniCare a percepção necessária para operar de forma autônoma em ambientes multi-andares.  
Ele garante que a troca de mapas ocorra apenas quando o robô realmente atingir o andar correto, aumentando a robustez e a segurança da navegação.