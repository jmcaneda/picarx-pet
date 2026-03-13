import os
import subprocess
import time

def play_direct(file):
    print(f"Intentando reproducir: {file}")
    # -D hw:2,0 obliga a ALSA a usar la Card 2 (tu Robot-Hat)
    # Si hw:2,0 falla, el error saldrá en pantalla
    subprocess.run(['aplay', '-D', 'plughw:2,0', '-q', f'/usr/share/sounds/alsa/{file}'])

print("--- TEST DIRECTO AL ROBOT-HAT (CARD 2) ---")

# 1. Forzamos el volumen de nuevo por si acaso
os.system("amixer -c 2 set 'robot-hat speaker' 90% > /dev/null")

sonidos = ['Front_Center.wav', 'Noise.wav']

for s in sonidos:
    play_direct(s)
    time.sleep(1)

print("--- Test finalizado ---")