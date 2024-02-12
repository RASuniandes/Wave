import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time

# Configurar la conexión serial (ajustar el puerto y el baudrate)
ser = serial.Serial('COM11', 115200, timeout=1)

# Crear la figura para la visualización 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Función para dibujar el avión
def draw_plane(yaw, pitch, roll):
    # Limpiar el gráfico anterior
    ax.clear()

    # Establecer los límites del gráfico
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Transformaciones para el yaw, pitch y roll
    # Aquí debes aplicar las transformaciones adecuadas
    # basadas en los ángulos yaw, pitch y roll

    # Dibujar el avión
    # Puedes usar plot para dibujar líneas que representen el avión
    ax.plot([0, 0.5], [0, 0], [0, 0], color='blue')  # Ejemplo de línea

    plt.draw()
    plt.pause(0.001)

# Loop principal
try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            
            # Esperar a recibir datos en el formato "yaw,pitch,roll"
            try:
                yaw, pitch, roll = map(float, line.split(','))
                draw_plane(yaw, pitch, roll)
            except ValueError:
                pass

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Programa terminado por el usuario")

finally:
    ser.close()
    plt.close()