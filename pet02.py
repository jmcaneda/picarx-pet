############################################################
# pet02.py
############################################################

import os
import time
from vilib import Vilib
from enum import Enum
from libs import hello_px, check_robot

# ============================================================
# CONSTANTES
# ============================================================

BALIZA_COLOR = "red"

PAN_MIN = -35
PAN_MAX = 35

TILT_MIN = -35
TILT_MAX = 35

CX = 160
CY = 120

""" Real
FAST_SPEED = 20 
SLOW_SPEED = 5
TURN_SPEED = 1 
"""

# simulado
FAST_SPEED = 0 
SLOW_SPEED = 0 
TURN_SPEED = 0

CAM_STEP = 4

SAFE_DISTANCE = 45      # cm
DANGER_DISTANCE = 25    # cm

LOG_PATH = os.path.join(os.path.dirname(__file__), "pet02.log")

# ============================================================
# CLASES
# ============================================================

class Estado(Enum):
    IDLE = 1
    RESET = 2
    SEARCH = 3
    RECENTER = 5
    TRACK = 10
    ERR = 89
    CHK = 99

class Cmd(Enum):
    STOP = 1
    FORWARD = 2
    FORWARD_SLOW = 3
    WHEELS_TURN_LEFT = 4
    WHEELS_TURN_RIGHT = 5
    BACKWARD = 6
    SCAPE = 8
    CAM_PAN_LEFT = 9
    CAM_PAN_RIGHT = 10
    CAM_TILT_TOP = 20
    CAM_TILT_BOTTOM = 30

class Det:
    """
    Representa una detección de la baliza.
    Incluye centroide, tamaño y utilidades para TRACK.
    """

    def __init__(self, n, x, y, w, h, cx=CX, cy=CY):
        self.n = n          # número de detecciones
        self.x = x          # centroide X
        self.y = y          # centroide Y
        self.w = w          # ancho detectado
        self.h = h          # alto detectado
        self.cx = cx        # centro ideal X
        self.cy = cy        # centro ideal Y

    # --- Propiedades útiles ---

    @property
    def area(self):
        return self.w * self.h

    @property
    def error_x(self):
        return self.x - self.cx

    @property
    def error_y(self):
        return self.y - self.cy

    @property
    def is_centered(self):
        if not self.valid_for_search:
            return False
        return abs(self.error_x) <= 40

    @property
    def valid(self):
        return (
            self.n > 0 and
            200 < self.area < 20000
        )

    @property
    def valid_for_search(self):
        return (
            self.n >= 1 and
            self.w > 10 and self.h > 10 and
            3000 < self.area < 25000 and
            100 < self.x < 620 and
            80 < self.y < 300
        )

    def __repr__(self):
        return (
            "Det("
            f"n={self.n}, "
            f"x={self.x}, y={self.y}, "
            f"w={self.w}, h={self.h}, "
            f"area={self.area}, "
            f"error_x={self.error_x}, error_y={self.error_y}, "
            f"is_centered={self.is_centered}, "
            f"valid={self.valid}, "
            f"valid_for_search={self.valid_for_search}"
            ")"
        )

class RobotState:
    def __init__(self):
        # TRACK anti-spam
        self.last_track_action = None
        self.last_error_x = None
        self.last_track_log = 0

        # RECENTER anti-spam
        self.last_recenter_action = None
        self.last_recenter_error_x = None
        self.last_recenter_log = 0

# ============================================================
# INICIALIZACIÓN
# ============================================================

def init_camera(px):
    # Iniciar cámara
    Vilib.camera_start(vflip=False, hflip=False)

    # Mostrar por web (local fallará si no hay GUI, es normal)
    Vilib.display(local=False, web=False)

    # Activar detección de color predefinido
    # (tu baliza funciona con "red")
    Vilib.color_detect(BALIZA_COLOR)

    time.sleep(0.5)

def init_wheels(px):
    px.Wheels_last_dir = 0

def init_internal_state(px):

    return Estado.IDLE, Cmd.STOP

def init_flags(px):
    px.last_log = None
    px.last_track_log = None
    px.estado_actual = None
    px.last_raw = {}
    px.last_state = None
    px.last_cmd = None
    px.last_raw_n = 0
    px.last_sec = "safe"
    px.dist = 999
    px.search_dir = 1      # 1 = derecha, -1 = izquierda
    px.last_pan = 0
    px.last_paneo = None
    px.last_tilt = 0
    px.search_cycles = 0   # cuántos barridos completos llevamos

# ============================================================
# LOGGING
# ============================================================

def log_event(px, estado, msg):
    # Si es igual al último mensaje, no lo repitas
    if px.last_log == (estado, msg):
        return

    px.last_log = (estado, msg)

    ts = time.strftime("%H:%M:%S", time.localtime())
    line = f"[{ts}] [{estado}] {msg}\n"

    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line)


# ============================================================
# ACCIONES BÁSICAS
# ============================================================

def stop(px):
    px.stop()

def forward(px, speed=FAST_SPEED):
    px.forward(speed)

def forward_slow(px, speed=SLOW_SPEED):
    px.forward(speed)

def backward(px, speed=SLOW_SPEED):
    px.backward(speed)

def turn_left(px, speed=TURN_SPEED):
    px.set_dir_servo_angle(-35)
    px.forward(speed)

def turn_right(px, speed=TURN_SPEED):
    px.set_dir_servo_angle(35)
    px.forward(speed)

def scape_danger(px, speed=SLOW_SPEED):
    px.set_dir_servo_angle(-35)
    px.backward(speed)
    time.sleep(0.5)
    px.set_dir_servo_angle(35)
    px.backward(speed)
    time.sleep(0.5)

# ============================================================
# MOVIMIENTOS DE CÁMARA SEGUROS
# ============================================================

def pan_right(px, step=CAM_STEP):
    new_angle = px.last_pan + step
    if new_angle >= PAN_MAX:
        new_angle = PAN_MAX
    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

def pan_left(px, step=CAM_STEP):
    new_angle = px.last_pan - step
    if new_angle <= PAN_MIN:
        new_angle = PAN_MIN
    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

def tilt_top(px, step=CAM_STEP):
    new_angle = px.last_tilt + step
    if new_angle >= TILT_MAX:
        new_angle = TILT_MAX
    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

def tilt_bottom(px, step=CAM_STEP):
    new_angle = px.last_tilt - step
    if new_angle <= TILT_MIN:
        new_angle = TILT_MIN
    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)


# ============================================================
# MAPEO DE COMANDOS
# ============================================================

def execute_motion(px, estado, cmd: Cmd, test_mode=False):
    """
    Ejecuta un comando de movimiento.
    En test_mode:
        - Permite pan/tilt y giros de ruedas.
        - Bloquea traslación (forward/backward) y solo imprime.
    """

    # ============================================================
    # MODO SIMULADO
    # ============================================================
    if test_mode:

        # --- Comandos permitidos en sim ---
        if cmd in (Cmd.CAM_PAN_LEFT, Cmd.CAM_PAN_RIGHT,
                   Cmd.CAM_TILT_TOP, Cmd.CAM_TILT_BOTTOM,
                   Cmd.WHEELS_TURN_LEFT, Cmd.WHEELS_TURN_RIGHT,
                   Cmd.STOP):

            # Ejecutar realmente (para ver movimiento en sim)
            try:
                if cmd == Cmd.STOP:
                    stop(px)
                elif cmd == Cmd.WHEELS_TURN_LEFT:
                    turn_left(px)
                elif cmd == Cmd.WHEELS_TURN_RIGHT:
                    turn_right(px)
                elif cmd == Cmd.CAM_PAN_LEFT:
                    pan_left(px)
                elif cmd == Cmd.CAM_PAN_RIGHT:
                    pan_right(px)
                elif cmd == Cmd.CAM_TILT_TOP:
                    tilt_top(px)
                elif cmd == Cmd.CAM_TILT_BOTTOM:
                    tilt_bottom(px)

                log_event(px, estado or Estado.CHK, f"[SIM] Ejecutado: {cmd.name}")
                return True

            except Exception as e:
                log_event(px, estado or Estado.ERR, f"[SIM] Error ejecutando {cmd}: {e}")
                return False

        # --- Comandos bloqueados en sim ---
        else:
            log_event(px, estado or Estado.CHK, f"[SIM] BLOQUEADO (solo imprimir): {cmd.name}")
            return True

    # ============================================================
    # MODO REAL
    # ============================================================
    try:
        if cmd == Cmd.STOP:
            stop(px)

        elif cmd == Cmd.FORWARD:
            forward(px, FAST_SPEED)

        elif cmd == Cmd.FORWARD_SLOW:
            forward_slow(px, SLOW_SPEED)

        elif cmd == Cmd.WHEELS_TURN_LEFT:
            turn_left(px)

        elif cmd == Cmd.WHEELS_TURN_RIGHT:
            turn_right(px)

        elif cmd == Cmd.BACKWARD:
            backward(px)

        elif cmd == Cmd.SCAPE:
            scape_danger(px, TURN_SPEED)

        # --- Cámara ---
        elif cmd == Cmd.CAM_PAN_LEFT:
            pan_left(px)

        elif cmd == Cmd.CAM_PAN_RIGHT:
            pan_right(px)

        elif cmd == Cmd.CAM_TILT_TOP:
            tilt_top(px)

        elif cmd == Cmd.CAM_TILT_BOTTOM:
            tilt_bottom(px)

        else:
            log_event(px, Estado.ERR, f"Comando desconocido: {cmd}")
            stop(px)
            return False

        return True

    except Exception as e:
        log_event(px, Estado.ERR, f"Error ejecutando {cmd}: {e}")
        stop(px)
        return False

# ============================================================
# SEGURIDAD
# ============================================================
def update_safety(px):
    raw = px.ultrasonic.read()
    if raw is None or raw < 0:
        return 999
    distance = round(raw, 2)
    return distance

def apply_safety(px, d, estado, accion):

    # --- Cooldown de seguridad ---
    #if time() - px.safety_cooldown < 1.0:
    #    return estado, Cmd.STOP

    # --- Zona segura ---
    if d > SAFE_DISTANCE:
        px.last_sec = "safe"
        return estado, accion

    # --- Zona de advertencia ---
    if SAFE_DISTANCE >= d > DANGER_DISTANCE:
        px.last_sec = "safe"
        return estado, accion

    # --- Zona de peligro crítico ---
    if d <= DANGER_DISTANCE:
        if px.last_sec != "critical":
            log_event(px, estado, f"[SEC] CRITICAL: object < {d} cm")
        px.last_sec = "critical"

        # SCAPE se ejecutará en el bucle principal
        #px.safety_cooldown = time()
        return Estado.RESET, Cmd.SCAPE

    return estado, accion


# ============================================================
# FUNCIONES
# ============================================================

def get_detection(px):
    params = Vilib.detect_obj_parameter

    # RAW para log
    raw = {
        "color_x": params.get("color_x", -1),
        "color_y": params.get("color_y", -1),
        "color_w": params.get("color_w", 0),
        "color_h": params.get("color_h", 0),
        "color_n": params.get("color_n", 0),
    }

    if raw["color_n"] > 0 and px.last_raw_n == 0:
        log_event(px, px.estado_actual, f"Det n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} area={raw['color_w']*raw['color_h']} x={raw['color_x']} y={raw['color_y']}")
    px.last_raw_n = 1 if raw["color_n"] > 0 else 0

    # Crear objeto Det
    det = Det(
        n = raw["color_n"],
        x = raw["color_x"],
        y = raw["color_y"],
        w = raw["color_w"],
        h = raw["color_h"]
    )

    return det

def search_see(px, det):
    
    return Estado.TRACK, Cmd.STOP

def search_not_see(px):

    # Paneo derecha
    if px.search_dir == 1:
        if px.last_pan < PAN_MAX:
            if px.last_paneo != "right":
                log_event(px, Estado.SEARCH, "Paneo → derecha")
                px.last_paneo = "right"
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT
        else:
            px.search_dir = -1
            log_event(px, Estado.SEARCH, "Cambio → izquierda")
            px.last_paneo = "left"
            return Estado.SEARCH, Cmd.CAM_PAN_LEFT

    # Paneo izquierda
    if px.search_dir == -1:
        if px.last_pan > PAN_MIN:
            if px.last_paneo != "left":
                log_event(px, Estado.SEARCH, "Paneo → izquierda")
                px.last_paneo = "left"
            return Estado.SEARCH, Cmd.CAM_PAN_LEFT
        else:
            px.search_dir = 1
            log_event(px, Estado.SEARCH, "Cambio → derecha")
            px.last_paneo = "right"
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT


# ============================================================
# ESTADOS
# ============================================================

def state_idle(px):
    log_event(px, Estado.IDLE, "Entrando en IDLE")
    return Estado.RESET, Cmd.STOP

def state_reset(px):
    log_event(px, Estado.RESET, "Entrando en RESET")
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    px.set_dir_servo_angle(0)
    px.last_pan = 0
    px.last_tilt = 0
    px.last_dir = 0

    return Estado.SEARCH, Cmd.STOP

def state_search(px, dist, estado, accion):
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, "Entrando en SEARCH")
        px.search_dir = 1
        px.last_paneo = None
        px.search_seen = 0

    px.last_state = Estado.SEARCH

    # Seguridad primero
    estado, accion = apply_safety(px, dist, estado, accion)
    if estado != Estado.SEARCH:
        return estado, accion

    det = get_detection(px)

    # --- Si vemos la baliza pero NO está centrada ---
    if det.valid_for_search and not det.is_centered:
        px.search_seen = 0

        # Si estamos en el límite → RECENTER
        if abs(px.last_pan) >= PAN_MAX:
            log_event(px, Estado.SEARCH, "Límite de pan → RECENTER")
            return Estado.RECENTER, Cmd.STOP

        # Corrección gruesa con cámara
        if det.error_x > 40:
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT

        if det.error_x < -40:
            return Estado.SEARCH, Cmd.CAM_PAN_LEFT

    # --- Si está centrada durante 2 frames → RECENTER ---
    if det.valid_for_search and det.is_centered:
        px.search_seen = getattr(px, "search_seen", 0) + 1

        if px.search_seen >= 2:
            log_event(px, Estado.SEARCH, "Baliza encontrada")
            return Estado.RECENTER, Cmd.STOP

    else:
        px.search_seen = 0

    # --- Si no vemos nada → paneo ---
    return search_not_see(px)

def state_recenter(px, dist, estado, accion, robot_state):
    # det = get_detection(px)

    # Entrada al estado
    if px.last_state != estado:
        log_event(px, estado, "Entrando en RECENTER")
        # log_event(px, estado, f"DEBUG det: valid={det.valid} area={det.area} x={det.x} pan={px.last_pan}")
    px.last_state = estado

    # Seguridad primero
    estado, accion = apply_safety(px, dist, estado, accion)
    if estado != Estado.RECENTER:
        return estado, accion

    det = get_detection(px)

    # Si no hay detección válida → SEARCH
    if not det.valid:
        log_event(px, Estado.RECENTER, "Perdida baliza → SEARCH")
        return Estado.SEARCH, Cmd.STOP

    # ============================================================
    # ANTI-SPAM RECENTER LOGIC
    # ============================================================
    now = time.time()

    if abs(det.error_x) <= 20:
        action = "CENTERED"
    elif det.error_x > 0:
        action = "TURN_RIGHT"
    else:
        action = "TURN_LEFT"

    should_log = (
        action != robot_state.last_recenter_action or
        robot_state.last_recenter_error_x is None or
        abs(det.error_x - robot_state.last_recenter_error_x) > 15 or
        now - robot_state.last_recenter_log > 0.3
    )

    if should_log:
        if action == "CENTERED":
            log_event(px, Estado.RECENTER, "Alineado ✔ (cuerpo + cámara)")
        elif action == "TURN_RIGHT":
            log_event(px, Estado.RECENTER, f"Giro cuerpo → derecha (error_x={det.error_x})")
        else:
            log_event(px, Estado.RECENTER, f"Giro cuerpo → izquierda (error_x={det.error_x})")

        robot_state.last_recenter_action = action
        robot_state.last_recenter_error_x = det.error_x
        robot_state.last_recenter_log = now

    # ============================================================
    # LÓGICA DE MOVIMIENTO
    # ============================================================

    # 1. Corrección gruesa → ruedas
    if det.error_x > 25:
        return Estado.RECENTER, Cmd.WHEELS_TURN_RIGHT

    if det.error_x < -25:
        return Estado.RECENTER, Cmd.WHEELS_TURN_LEFT

    # 2. Corrección fina → cámara
    if det.error_x > 8:
        return Estado.RECENTER, Cmd.CAM_PAN_RIGHT

    if det.error_x < -8:
        return Estado.RECENTER, Cmd.CAM_PAN_LEFT

    # 3. Alineado → pasar a TRACK
    return Estado.TRACK, Cmd.FORWARD_SLOW

def state_track(px, dist, estado, accion, robot_state):
    det = get_detection(px)

    # Entrada al estado
    if px.last_state != estado:
        log_event(px, estado, "Entrando en TRACK")
        log_event(px, estado, f"DEBUG det: valid={det.valid} area={det.area} x={det.x} pan={px.last_pan}")
    px.last_state = estado

    # Seguridad primero
    estado, accion = apply_safety(px, dist, estado, accion)
    if estado != Estado.TRACK:
        return estado, accion

    # Si no hay detección válida → SEARCH
    if not det.valid:
        log_event(px, Estado.TRACK, "Perdida baliza → SEARCH")
        return Estado.SEARCH, Cmd.STOP

    # ============================================================
    # ANTI-SPAM TRACK LOGIC
    # ============================================================
    now = time.time()

    if abs(det.error_x) > 20:
        action = "CORRECT"
    else:
        action = "FORWARD"

    should_log = (
        action != robot_state.last_track_action or
        robot_state.last_error_x is None or
        abs(det.error_x - robot_state.last_error_x) > 15 or
        now - robot_state.last_track_log > 0.3
    )

    if should_log:
        if action == "FORWARD":
            log_event(px, Estado.TRACK, "Avanzando hacia baliza")
        else:
            log_event(px, Estado.TRACK, f"Corrigiendo error_x={det.error_x}")

        robot_state.last_track_action = action
        robot_state.last_error_x = det.error_x
        robot_state.last_track_log = now

    # ============================================================
    # LÓGICA DE MOVIMIENTO
    # ============================================================

    # 1. Corrección gruesa → ruedas
    if det.error_x > 25:
        return Estado.TRACK, Cmd.WHEELS_TURN_RIGHT

    if det.error_x < -25:
        return Estado.TRACK, Cmd.WHEELS_TURN_LEFT

    # 2. Corrección fina → cámara
    if det.error_x > 8:
        return Estado.TRACK, Cmd.CAM_PAN_RIGHT

    if det.error_x < -8:
        return Estado.TRACK, Cmd.CAM_PAN_LEFT

    # 3. Centrado → avanzar
    return Estado.TRACK, Cmd.FORWARD_SLOW

# ============================================================
# BUCLE PRINCIPAL
# ============================================================

def pet_mode(px, test_mode):
    print("🐾 Pet02 mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet02.log ===\n")

    hello_px(px)
    init_camera(px)
    init_wheels(px)
    init_flags(px)
    estado, accion = init_internal_state(px)
    check_robot(px,log_event)
    state = RobotState()
    log_event(px, estado, "Inicio del sistema")

    while True:
        px.estado_actual = estado
        px.dist = update_safety(px)

        if estado == Estado.IDLE:
            estado, accion = state_idle(px)
        elif estado == Estado.RESET:
            estado, accion = state_reset(px)
        elif estado == Estado.SEARCH:
            estado, accion = state_search(px, px.dist, estado, accion)
        elif estado == Estado.RECENTER:
            estado, accion = state_recenter(px, px.dist, estado, accion, state)
        elif estado == Estado.TRACK:
            estado, accion = state_track(px, px.dist, estado, accion, state)

        if px.last_cmd != accion:
            log_event(px, estado, f"CMD {accion.name}")
        px.last_cmd = accion

        estado, accion = apply_safety(px, px.dist, estado, accion)

        execute_motion(px, estado, accion, test_mode)

        time.sleep(0.5)

# ============================================================
# ENTRYPOINT
# ============================================================

if __name__ == "__main__":
    import sys
    from picarx import Picarx
    from libs import get_px

    modo = "sim"
    if len(sys.argv) > 1:
        modo = sys.argv[1].lower()

    test_mode = (modo != "real")
    print(f"Arrancando pet02.py en modo: {modo.upper()}")

    px = get_px()
    px.test_mode = test_mode
    pet_mode(px, test_mode=test_mode)
