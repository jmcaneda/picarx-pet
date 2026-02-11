############################################################
# pet01.py - Arquitectura modular y limpia para PiCar-X
############################################################

import os
import time

from vilib import Vilib
from picarx import Picarx

from enum import Enum


# ============================================================
# CONSTANTES
# ============================================================

BALIZA_COLOR = "red"

PAN_MIN = -45
PAN_MAX = 45

TILT_MIN = -20
TILT_MAX = 20

# Centro de la imagen (ajusta si tu cámara es distinta)
CX = 160
CY = 120

LOG_PATH = os.path.join(os.path.dirname(__file__), "pet01.log")

# ============================================================
# CLASES
# ============================================================

class Det:
    def __init__(self, x, y, w, h, cx=CX, cy=CY):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.cx = cx
        self.cy = cy

    def area(self):
        return self.w * self.h

    def error_x(self):
        return self.x - self.cx

    def error_y(self):
        return self.y - self.cy

    def is_centered(self, tol=25):
        return abs(self.error_x()) <= tol

    def __repr__(self):
        return f"Det(x={self.x}, y={self.y}, w={self.w}, h={self.h}, area={self.area()})"


class Estado(Enum):
    IDLE = 1
    RESET = 2
    SEARCH = 3
    TRACK = 4
    CHK = 99

# ============================================================
# LOGGING
# ============================================================

def log_event(px, estado, mensaje=""):

    if estado is None:
        estado_str = "NONE"
    else:
        estado_str = estado.name

    timestamp = time.strftime("%H:%M:%S", time.localtime())
    pan = getattr(px, "last_pan", 0)
    tilt = getattr(px, "last_tilt", 0)

    line = f"[{timestamp}] [{estado_str}] PAN={pan:.1f} TILT={tilt:.1f} {mensaje}"

    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")

# ============================================================
# CÁMARA
# ============================================================

def init_camera(px):
    # Iniciar cámara
    Vilib.camera_start(vflip=False, hflip=False)

    # Mostrar por web (local fallará si no hay GUI, es normal)
    Vilib.display(local=False, web=True)

    # Activar detección de color predefinido
    # (tu baliza funciona con "red")
    Vilib.color_detect(BALIZA_COLOR)

    time.sleep(1)

# ============================================================
# DETECCIÓN CRUDA
# ============================================================

def get_detection(px):
    params = Vilib.detect_obj_parameter

    # Log de VILIB RAW solo cuando cambie
    raw = {
        "color_x": params.get("color_x"),
        "color_y": params.get("color_y"),
        "color_w": params.get("color_w"),
        "color_h": params.get("color_h"),
        "color_n": params.get("color_n"),
    }

    log_event(px, px.estado_actual, f"VILIB RAW: {raw}")

    px.last_raw = raw

    n = params.get("color_n", 0)
    w = params.get("color_w", 0)
    h = params.get("color_h", 0)

    # Sin detección
    if n == 0 or w < 5 or h < 5:
        return None

    x = params.get("color_x", -1)
    y = params.get("color_y", -1)

    return Det(x, y, w, h, cx=CX, cy=CY)


# ============================================================
# MEMORIA DE DETECCIÓN
# ============================================================

def update_beacon_memory(px, det):
    if det is not None:
        px.last_det = det
        px.last_det_age = 0
    else:
        px.last_det_age += 1


def is_beacon_visible(px):
    det = get_detection(px)
    update_beacon_memory(px, det)

    # Memoria de 5 frames para evitar parpadeos
    return det is not None or px.last_det_age <= 5


def get_beacon_memory(px):
    return px.last_det if px.last_det_age <= 5 else None

# ============================================================
# ESTADO INTERNO
# ============================================================

def init_internal_state(px):
    px.last_det = None
    px.last_det_age = 999

    px.search_fail_ticks = 0

    px.last_safety_state = None
    px.safety_buffer = []

    px.last_pan = 0
    px.last_tilt = 0
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

    px.last_raw = None

    estado = Estado.IDLE

    accion = "STOP"

    return estado, accion

# ============================================================
# UTILES
# ============================================================

def clamp_pan(angle):
    return max(PAN_MIN, min(PAN_MAX, angle))

def clamp_tilt(angle):
    return max(TILT_MIN, min(TILT_MAX, angle))

# ============================================================
# SEGURIDAD FINAL
# ============================================================

def apply_safety(px, cmd):
    if getattr(px, "last_safety_state", None) == "CRITICAL":
        return "STOP"
    return cmd

# ============================================================
# ESTADOS
# ============================================================

def state_idle(px):
    log_event(px, Estado.IDLE, "Entrando en IDLE")
    return Estado.RESET, "STOP"


def state_reset(px):
    log_event(px, Estado.RESET, "RESET: limpiando estado interno")

    px.last_det = None
    px.last_det_age = 999

    px.track_escape_ticks = 0
    px.search_fail_ticks = 0

    px.last_safety_state = None
    px.safety_buffer = []

    # Cámara centrada
    px.last_pan = 0
    px.last_tilt = 0
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

    # Dirección de barrido para SEARCH
    px.search_dir = +1

    return Estado.SEARCH, "STOP"


def state_search(px):
    """
    SEARCH con dos modos:
      - STATIC: mueve la cámara para encontrar/centrar la baliza
      - DYNAMIC: mueve el cuerpo cuando la cámara ya está centrada
    """

    # Reinicio si venimos de TRACK
    if getattr(px, "prev_state", None) == Estado.TRACK:
        px.search_mode = "STATIC"
        px.search_fail_ticks = 0
        px.search_centered_frames = 0
        log_event(px, Estado.SEARCH, "Reiniciando SEARCH_STATIC tras TRACK")

    # Inicialización
    if not hasattr(px, "search_mode"):
        px.search_mode = "STATIC"
        px.search_fail_ticks = 0
        px.search_dir = +1
        px.search_track_stable = 0
        px.search_centered_frames = 0
        log_event(px, Estado.SEARCH, "Entrando en SEARCH_STATIC")

    visible = is_beacon_visible(px)
    det = get_beacon_memory(px)

    # ============================================================
    # 1. Entrada a TRACK con histéresis fuerte
    # ============================================================
    if visible and det is not None and det.area() > 3000 and abs(det.error_x()) < 10:
        px.search_track_stable += 1
    else:
        px.search_track_stable = 0

    if px.search_track_stable >= 5:
        log_event(px, Estado.SEARCH, "Baliza estable → TRACK")
        px.search_track_stable = 0
        return Estado.TRACK, "STOP"

    px.search_fail_ticks += 1

    # ============================================================
    # STATIC — controla la cámara
    # ============================================================
    if px.search_mode == "STATIC":
        log_event(px, Estado.SEARCH, "SEARCH_STATIC")

        if det is not None:
            err = det.error_x()

            # Mover cámara para centrar
            if err > 20:
                new_pan = clamp_pan(px.last_pan + 5)
                px.set_cam_pan_angle(new_pan)
                px.last_pan = new_pan

            elif err < -20:
                new_pan = clamp_pan(px.last_pan - 5)
                px.set_cam_pan_angle(new_pan)
                px.last_pan = new_pan

            else:
                # Histéresis para pasar a DYNAMIC
                px.search_centered_frames += 1
                if px.search_centered_frames >= 3:
                    px.search_mode = "DYNAMIC"
                    px.search_centered_frames = 0
                    log_event(px, Estado.SEARCH, "Cámara centrada → SEARCH_DYNAMIC")
                    return Estado.SEARCH, "STOP"

        else:
            # Barrido panorámico
            new_pan = px.last_pan + px.search_dir * 5

            if new_pan > PAN_MAX:
                new_pan = PAN_MAX
                px.search_dir = -1
            elif new_pan < PAN_MIN:
                new_pan = PAN_MIN
                px.search_dir = +1

            new_pan = clamp_pan(new_pan)
            px.set_cam_pan_angle(new_pan)
            px.last_pan = new_pan

        # Escape STATIC → DYNAMIC
        if px.search_fail_ticks > 40:
            px.search_mode = "DYNAMIC"
            px.search_fail_ticks = 0
            log_event(px, Estado.SEARCH, "Sin pistas → SEARCH_DYNAMIC")
            return Estado.SEARCH, "STOP"

        return Estado.SEARCH, "STOP"

    # ============================================================
    # DYNAMIC — controla el cuerpo
    # ============================================================
    if px.search_mode == "DYNAMIC":
        log_event(px, Estado.SEARCH, "SEARCH_DYNAMIC")

        if det is not None:
            err = det.error_x()

            # Solo mover cuerpo si cámara está centrada
            if abs(px.last_pan) < 10:
                if err > 30:
                    return Estado.SEARCH, "TURN_RIGHT"
                elif err < -30:
                    return Estado.SEARCH, "TURN_LEFT"
                else:
                    return Estado.SEARCH, "FORWARD_SLOW"
            else:
                return Estado.SEARCH, "STOP"

        # Barrido corporal
        if px.search_fail_ticks % 20 == 0:
            return Estado.SEARCH, "TURN_RIGHT" if px.search_dir > 0 else "TURN_LEFT"

        # Escape DYNAMIC → STATIC
        if px.search_fail_ticks > 60:
            px.search_mode = "STATIC"
            px.search_fail_ticks = 0
            log_event(px, Estado.SEARCH, "Sin detección → SEARCH_STATIC")
            return Estado.SEARCH, "STOP"

        return Estado.SEARCH, "FORWARD_SLOW"


def state_track(px):
    log_event(px, Estado.TRACK, "TRACK")

    # Inicialización
    if not hasattr(px, "track_stable_frames"):
        px.track_stable_frames = 0
    if not hasattr(px, "track_lost_frames"):
        px.track_lost_frames = 0

    visible = is_beacon_visible(px)
    det = get_beacon_memory(px)

    # ============================================================
    # 1. Pérdida de baliza → volver inmediatamente a SEARCH
    # ============================================================
    if not visible or det is None:
        log_event(px, Estado.TRACK, "Baliza perdida → SEARCH")
        px.track_lost_frames = 0
        px.track_stable_frames = 0
        return Estado.SEARCH, "STOP"

    px.track_lost_frames = 0

    error_x = det.error_x()
    area = det.area()

    # ============================================================
    # 2. RECENTER — mover cámara hacia el centro
    # ============================================================
    if abs(px.last_pan) > 10:
        log_event(px, Estado.TRACK, "RECENTER_BODY")

        if px.last_pan > 0:
            new_pan = clamp_pan(px.last_pan - 5)
        else:
            new_pan = clamp_pan(px.last_pan + 5)

        px.set_cam_pan_angle(new_pan)
        px.last_pan = new_pan

        if abs(px.last_pan) < 5:
            px.set_cam_pan_angle(0)
            px.last_pan = 0
            log_event(px, Estado.TRACK, "RECENTER completado")

        return Estado.TRACK, "STOP"

    # ============================================================
    # 3. Histéresis de estabilidad
    # ============================================================
    if abs(error_x) < 20:
        px.track_stable_frames += 1
    else:
        px.track_stable_frames = 0

    # ============================================================
    # 4. Zona muerta
    # ============================================================
    if abs(error_x) < 15:
        if area < 4000:
            return Estado.TRACK, "FORWARD"
        elif area < 8000:
            return Estado.TRACK, "FORWARD_SLOW"
        else:
            return Estado.TRACK, "STOP"

    # ============================================================
    # 5. Suavizado del giro
    # ============================================================
    if px.track_stable_frames < 2:
        return Estado.TRACK, "STOP"

    # ============================================================
    # 6. Giro según error_x
    # ============================================================
    if error_x > 25:
        return Estado.TRACK, "TURN_RIGHT"
    elif error_x < -25:
        return Estado.TRACK, "TURN_LEFT"

    return Estado.TRACK, "STOP"


# ============================================================
# BUCLE PRINCIPAL
# ============================================================

def pet_mode(px, test_mode):
    print("🐾 Pet01 mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet01.log ===\n")

    hello_px(px)
    init_camera(px)
    px.logger = log_event
    estado, accion = init_internal_state(px)
    check_robot(px, logger=log_event)

    while True:
        px.estado_actual = estado
        update_safety(px, logger=log_event)

        if estado == Estado.IDLE:
            estado, accion = state_idle(px)
        elif estado == Estado.RESET:
            estado, accion = state_reset(px)
        elif estado == Estado.SEARCH:
            estado, accion = state_search(px)
        elif estado == Estado.TRACK:
            estado, accion = state_track(px)

        accion = apply_safety(px, accion)

        log_event(px, estado, f"CMD {accion}")

        execute_motion(px, accion, test_mode)

        time.sleep(0.05)

# ============================================================
# ENTRYPOINT
# ============================================================

if __name__ == "__main__":
    import sys
    from picarx import Picarx

    modo = "sim"
    if len(sys.argv) > 1:
        modo = sys.argv[1].lower()

    test_mode = (modo != "real")
    print(f"Arrancando pet01.py en modo: {modo.upper()}")

    px = Picarx()
    px.test_mode = test_mode 
    pet_mode(px, test_mode=test_mode)
