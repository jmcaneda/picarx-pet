# sound.py
import os
from time import sleep
from picarx.music import Music

music = Music()

# Carpeta donde está este archivo
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SOUNDS_DIR = os.path.join(BASE_DIR, "sounds")

def _play(filename, volume=20, delay=0.05):
    path = os.path.join(SOUNDS_DIR, filename)

    if not os.path.exists(path):
        print(f"[sound] Archivo no encontrado: {path}")
        return

    music.music_set_volume(volume)
    music.sound_play(path)
    sleep(delay)

def sound_normal():
    _play("car-start-engine.wav")

def sound_caution():
    _play("car-double-horn.wav")

def sound_danger():
    _play("car-double-horn.wav")

def sound_angry():
    _play("sounds_angry.wav")
