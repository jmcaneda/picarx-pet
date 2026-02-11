# libs.py

from picarx import Picarx
from time import sleep
import os
import glob

# Instancia global única del PiCar-X
_px_instance = None


def clear_screen():
    """Limpia la pantalla de la terminal."""
    print("\033[H\033[J", end='')


def cleanup_fifos():
    """Elimina FIFOs residuales creados por gpiozero/lgpio."""
    for fifo in glob.glob(".lgd-*"):
        try:
            os.remove(fifo)
        except:
            pass


def get_px():
    """
    Inicializa Picarx de forma segura.
    Garantiza que SOLO se cree una instancia global.
    Evita múltiples resets del MCU (GPIO5), que causan bloqueos.
    """
    global _px_instance

    if _px_instance is None:
        px = Picarx()

        # Posiciones seguras iniciales
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)
        px.set_dir_servo_angle(0)

        _px_instance = px

    return _px_instance


def hello_px(px):
    """
    Cabecea la cámara, a modo de saludo.
    """
    if px:
        # Movimiento vertical (tilt)
        px.set_cam_tilt_angle(20)
        sleep(0.2)
        px.set_cam_tilt_angle(0)
        sleep(0.2)
        px.set_cam_tilt_angle(-20)
        sleep(0.2)
        px.set_cam_tilt_angle(0)
        sleep(0.2)

        # Movimiento horizontal (pan)
        px.set_cam_pan_angle(20)
        sleep(0.2)
        px.set_cam_pan_angle(0)
        sleep(0.2)
        px.set_cam_pan_angle(-20)
        sleep(0.2)
        px.set_cam_pan_angle(0)
        sleep(0.2)

        # Movimiento Wheels
        px.set_dir_servo_angle(35)
        sleep(0.2)
        px.set_dir_servo_angle(0)
        sleep(0.2)
        px.set_dir_servo_angle(-35)
        sleep(0.2)
        px.set_dir_servo_angle(0)
        sleep(0.2)

        return True
    else:
        return False

def check_robot(px, logger=None):
    """
    Muestra el estado interno del robot.
    Si se pasa un logger (log_event), escribe en pet.log.
    Si no, imprime por pantalla.
    """

    def out(msg):
        if logger:
            logger(px, "Estado.CHK", msg)
        else:
            print(msg)

    out("[CHK] ==============================")
    out("[CHK]    🧪 ESTADO DEL ROBOT")
    out("[CHK] ==============================")

    # --- Cámara ---
    out(f"[CHK] PAN cámara:            {getattr(px, 'last_pan', 0):.1f}°")
    out(f"[CHK] TILT cámara:           {getattr(px, 'last_tilt', 0):.1f}°")

    # --- Wheels ---
    out(f"[CHK] State wheels:     {getattr(px, 'last_dir', 0)}")

    out("[CHK] ==============================")
