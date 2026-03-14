import os
import subprocess
from picarx.music import Music
from time import sleep

def main():
    print("--- Intentando despertar el audio ---")
    
    try:
        # 1. Inicializamos la clase (esto a veces abre el canal)
        m = Music()
        m.music_set_volume(80)
        print("Librería inicializada...")
        
        # 2. Intentamos un sonido nativo corto
        # (Usa un archivo que sepas que existe en tu carpeta de picar-x)
        print("Probando sonido nativo...")
        m.sound_play('../sounds/car-double-horn.wav')
        sleep(1)
        
    except Exception as e:
        print(f"La librería falló sin sudo: {e}")
        print("Intentando bypass directo...")

    # 3. El comando que nos funcionaba, pero ahora con el canal 'abierto'
    print("Ejecutando aplay directo...")
    os.system("aplay -D plughw:2,0 -q /usr/share/sounds/alsa/Front_Center.wav")

if __name__ == "__main__":
    main()