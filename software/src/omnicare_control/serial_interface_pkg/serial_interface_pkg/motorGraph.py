import serial
import struct
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configurar a porta serial
ser = serial.Serial('/dev/usb-user', 115200, timeout=1)

# Buffers de dados
max_len = 250
velocidade = deque(maxlen=max_len)
erro = deque(maxlen=max_len)
pwm = deque(maxlen=max_len)
amostras = deque(maxlen=max_len)

contador = 0

def ler_dados():
    global contador
    while True:
        try:
            linha = ser.readline().decode('utf-8').strip()  # Lê 4 bytes enviados pelo STM32
            partes = list(map(int, linha.split()))
            if len(partes) == 4:
                v, e, dt, p = map(int, partes)
                velocidade.append(v)
                erro.append(e)
                pwm.append(p)
                amostras.append(contador)
                contador += 1

            ser.reset_input_buffer()
        except Exception as ex:
            print("Erro na leitura:", ex)



# Thread de leitura dos dados da serial
t = threading.Thread(target=ler_dados, daemon=True)
t.start()

# Atualização do gráfico
def animar(i):
    # print('entrou')
    ax1.clear()
    ax2.clear()
    ax3.clear()

    ax1.plot(amostras, velocidade, label="Velocidade")
    ax2.plot(amostras, pwm, label="PWM")
    ax3.plot(amostras, erro, label="Erro", color="red")

    ax1.set_ylabel("Velocidade / PWM")
    ax2.set_ylabel("Erro")
    ax3.set_xlabel("Amostras")
    ax1.legend()
    ax2.legend()
    ax3.legend()
    
    ax1.grid(True)
    ax2.grid(True)
    ax3.grid(True)

# Setup do gráfico
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
ani = animation.FuncAnimation(fig, animar, interval=100)
plt.tight_layout()

# Thread para envio de comandos ao microcontrolador
def entrada_usuario():
    while True:
        comando = input("Digite comando (char): ")
        ser.write(comando.encode())

te = threading.Thread(target=entrada_usuario, daemon=True)
te.start()

# Inicia o gráfico
plt.show()