############################################################
# pet02.py
############################################################

import os
import time
from vilib import Vilib
from enum import Enum
from libs import hello_px, check_robot
# from picarx.music import Music

# music = Music()

# ============================================================
# CONSTANTES
# ============================================================

BALIZA_COLOR = "red"

PAN_MIN = -35
PAN_MAX = 35

TILT_MIN = -20
TILT_MAX = 20

SERVO_ANGLE_MIN = -30
SERVO_ANGLE_MAX = 30

CX = 320
CY = 240

FAST_SPEED = 20 
SLOW_SPEED = 5
TURN_SPEED = 1 

CAM_STEP = 4

SAFE_DISTANCE = 999
WARNING_DISTANCE = 35.0
DANGER_DISTANCE = 20.0

LOG_PATH = os.path.join(os.path.dirname(__file__), "pet02.log")

# SOUNDS_DIR = "/home/jmcaneda/picarx-projects/autonomous/sounds"

# ============================================================
# CLASES
# ============================================================

class Estado(Enum):
    IDLE = 1
    RESET = 2
    SEARCH = 3
    RECENTER = 5
    TRACK = 10
    NEAR = 15
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
    KEEP_ALIVE = 99 # Comando ficticio para mantener maniobra activa sin cambiar estado

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
        """
        Centrado geométrico puro.
        No depende de validadores.
        """
        return abs(self.error_x) <= 40

    @property
    def valid_for_search(self):
        """
        Cualquier detección razonable que merezca ser tenida en cuenta.
        Amplio y tolerante.
        """
        if self.w == 0 or self.h == 0 or self.n == 0:
            return False

        return (
            self.n >= 1 and
            10 < self.w < 640 and
            10 < self.h < 480 and
            200 < self.area < 240000 and
            0 < self.x < 640 and
            0 < self.y < 480
        )

    @property
    def valid_for_track(self):
        """
        Detección suficientemente buena como para mover el chasis.
        Más estricta que search, pero no tan exigente como NEAR.
        """
        if not self.valid_for_search:
            return False

        # Evitar cosas demasiado pequeñas o gigantescas
        if not (800 < self.area < 160000):
            return False

        # Mantenernos dentro de una zona útil (no pegado al borde)
        if not (40 < self.x < 600):
            return False

        return True

    @property
    def valid_for_near(self):
        """
        Detección de 'muy cerca y bastante centrada'.
        Subconjunto de valid_for_track.
        """
        if not self.valid_for_track:
            return False

        # 1. Área realmente grande (cerca de verdad)
        if self.area < 12000:
            return False

        # 2. Centrado horizontal más estricto
        if abs(self.error_x) > 40:
            return False

        return True

    def __repr__(self):
        return (
            "Det("
            f"n={self.n}, "
            f"x={self.x}, y={self.y}, "
            f"w={self.w}, h={self.h}, "
            f"area={self.area}, "
            f"error_x={self.error_x}, error_y={self.error_y}, "
            f"is_centered={self.is_centered}, "
            f"valid_for_search={self.valid_for_search}, "
            f"valid_for_track={self.valid_for_track}, "
            f"valid_for_near={self.valid_for_near}"
            ")"
        )

class RobotState:
    def __init__(self):

        # ============================================================
        # SEARCH
        # ============================================================
        self.search_lost_frames = 0          # frames sin detección válida
        self.search_found_frames = 0         # frames con detección válida
        self.search_cam_dir = 1              # dirección del barrido PAN
        self.search_wheels_dir = 1           # dirección del giro del chasis
        self.search_edge_frames = 0          # cuántos frames la baliza está en el borde

        # ============================================================
        # RECENTER
        # ============================================================
        self.recenter_centered_frames = 0    # frames centrados
        self.recenter_lost_frames = 0        # frames sin detección
        self.just_recentered = None          # cooldown tras RECENTER

        # ============================================================
        # TRACK
        # ============================================================
        self.track_lost_frames = 0           # frames sin detección válida
        self.track_centered_frames = 0       # frames centrado durante TRACK

        # ============================================================
        # NEAR
        # ============================================================
        self.near_enter_frames = 0
        self.near_exit_frames = 0
        self.near_lost_frames = 0
        self.near_done_backward = False
        self.near_cooldown = None
        self.near_did_yes = False

        # Animación YES
        self.yes_step = 0
        self.yes_next_time = 0.0

        # ============================================================
        # SCAPE
        # ============================================================
        self.is_escaping = False
        self.escape_end_time = 0
        self.last_sec_active = False

# ============================================================
# INICIALIZACIÓN
# ============================================================

def init_camera(px):
    # Iniciar cámara
    Vilib.camera_start(vflip=False, hflip=False)

    # Mostrar por web (local fallará si no hay GUI, es normal)
    Vilib.display(local=False, web=False)

    # Activar detección de color predefinido
    Vilib.color_detect(BALIZA_COLOR)

    time.sleep(0.5)

def init_internal_state(px):
    return Estado.IDLE, Cmd.STOP

def init_flags(px):
    # ============================================================
    # LOGGING
    # ============================================================
    px.last_log = None

    # ============================================================
    # ESTADOS
    # ============================================================
    px.last_state = None          # último estado ejecutado
    px.estado_actual = None       # estado actual del robot (para logging)
    px.last_cmd = None            # último comando enviado al robot

    # ============================================================
    # DETECCIÓN (últimos valores vistos)
    # ============================================================
    px.last_det = None            # última detección válida
    px.last_error_x = 0           # último error horizontal
    px.last_error_y = 0           # último error vertical
    px.last_area = 0              # última área detectada

    # ============================================================
    # CÁMARA
    # ============================================================
    px.last_pan = 0               # ángulo actual de PAN
    px.last_tilt = 0              # ángulo actual de TILT (si lo usas)
    
    # ============================================================
    # DIRECCIÓN DEL CHASIS
    # ============================================================
    px.dir_current_angle = 0      # ángulo actual del servo de dirección


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
# ACCIONES BÁSICAS v3 — Movimiento fluido, seguro y con retorno
# ============================================================

def stop(px):
    """
    Detiene el robot sin tocar el servo de dirección.
    Devuelve True si realmente se detuvo.
    """
    if px.last_cmd == "STOP":
        return False  # comando redundante

    px.stop()
    px.last_cmd = "STOP"
    return True


def forward(px, speed=FAST_SPEED):
    """
    Avance normal.
    """
    if px.last_cmd == ("FWD", speed):
        return False

    px.forward(speed)
    px.last_cmd = ("FWD", speed)
    return True


def forward_slow(px, speed=SLOW_SPEED):
    """
    Avance suave para TRACK y SEARCH.
    """
    if px.last_cmd == ("FWD_SLOW", speed):
        return False

    px.forward(speed)
    px.last_cmd = ("FWD_SLOW", speed)
    return True


def backward(px, speed=SLOW_SPEED):
    """
    Retroceso seguro.
    """
    if px.last_cmd == ("BACK", speed):
        return False

    px.backward(speed)
    px.last_cmd = ("BACK", speed)
    return True


# ------------------------------------------------------------
# GIRO CONTINUO REAL
# ------------------------------------------------------------

def turn_left(px, speed=TURN_SPEED):
    """
    Gira suavemente a la izquierda.
    No reescribe el servo si ya está en ese ángulo.
    """
    if px.dir_current_angle != SERVO_ANGLE_MIN:
        px.set_dir_servo_angle(SERVO_ANGLE_MIN)
        px.dir_current_angle = SERVO_ANGLE_MIN

    if px.last_cmd == ("TURN_LEFT", speed):
        return False

    px.forward(speed)
    px.last_cmd = ("TURN_LEFT", speed)
    return True


def turn_right(px, speed=TURN_SPEED):
    """
    Gira suavemente a la derecha.
    """
    if px.dir_current_angle != SERVO_ANGLE_MAX:
        px.set_dir_servo_angle(SERVO_ANGLE_MAX)
        px.dir_current_angle = SERVO_ANGLE_MAX

    if px.last_cmd == ("TURN_RIGHT", speed):
        return False

    px.forward(speed)
    px.last_cmd = ("TURN_RIGHT", speed)
    return True


# ------------------------------------------------------------
# SCAPE
# ------------------------------------------------------------

def scape_danger(px, robot_state, speed=SLOW_SPEED):
    """
    Maniobra de evasión determinista:
    - Retrocede recto
    - Gira el chasis un ángulo fijo
    - Retrocede un poco más
    - Centra cámara y ruedas
    - Devuelve control limpio a SEARCH
    """

    # ------------------------------------------------------------
    # FASE 1: INICIO
    # ------------------------------------------------------------
    if not robot_state.is_escaping:
        log_event(px, "SEC", "¡ESCAPE ACTIVO! Retrocediendo...")

        # 1. Retroceso inicial
        px.backward(speed + 5)

        # 2. Elegir dirección fija de escape
        #    Alternamos para evitar patrones repetitivos
        robot_state.search_wheels_dir *= -1
        escape_angle = SERVO_ANGLE_MAX * robot_state.search_wheels_dir

        px.set_dir_servo_angle(escape_angle)
        px.dir_current_angle = escape_angle

        # 3. Duración fija
        robot_state.escape_end_time = time.time() + 1.2
        robot_state.is_escaping = True
        robot_state.last_sec_active = True

        return False

    # ------------------------------------------------------------
    # FASE 2: MANTENER ESCAPE
    # ------------------------------------------------------------
    dist = update_safety(px)

    # Si el obstáculo sigue muy cerca, extendemos un poco
    if 0 < dist < 15:
        robot_state.escape_end_time = time.time() + 0.3

    # ------------------------------------------------------------
    # FASE 3: FINALIZAR
    # ------------------------------------------------------------
    if time.time() >= robot_state.escape_end_time:
        px.stop()

        # Centrar ruedas
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        # Centrar cámara
        px.set_cam_pan_angle(0)
        px.last_pan = 0

        # Reset de flags
        robot_state.is_escaping = False
        robot_state.search_lost_frames = 0
        robot_state.search_found_frames = 0
        robot_state.search_edge_frames = 0

        log_event(px, "SEC", "Maniobra terminada.")
        return True

    return False


# ============================================================
# MOVIMIENTOS DE CÁMARA SEGUROS
# ============================================================

def pan_right(px, step=CAM_STEP):
    """
    Mueve PAN a la derecha.
    Devuelve:
        +1 si se movió
         0 si ya estaba en el límite
    """
    new_angle = px.last_pan + step
    if new_angle >= PAN_MAX:
        px.last_pan = PAN_MAX
        px.set_cam_pan_angle(px.last_pan)
        return 0  # no hubo movimiento real

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)
    return 1


def pan_left(px, step=CAM_STEP):
    """
    Mueve PAN a la izquierda.
    Devuelve:
        +1 si se movió
         0 si ya estaba en el límite
    """
    new_angle = px.last_pan - step
    if new_angle <= PAN_MIN:
        px.last_pan = PAN_MIN
        px.set_cam_pan_angle(px.last_pan)
        return 0

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)
    return 1


def tilt_top(px, step=CAM_STEP):
    new_angle = px.last_tilt + step
    if new_angle >= TILT_MAX:
        px.last_tilt = TILT_MAX
        px.set_cam_tilt_angle(px.last_tilt)
        return 0

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)
    return 1


def tilt_bottom(px, step=CAM_STEP):
    new_angle = px.last_tilt - step
    if new_angle <= TILT_MIN:
        px.last_tilt = TILT_MIN
        px.set_cam_tilt_angle(px.last_tilt)
        return 0

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)
    return 1

# ============================================================
# MAPEO DE COMANDOS — v3 (determinista, seguro, sin redundancias)
# ============================================================

def execute_motion(px, estado, cmd: Cmd, robot_state, test_mode=False):

    # ------------------------------------------------------------
    # 0. PRIORIDAD ABSOLUTA: SCAPE
    # ------------------------------------------------------------
    if robot_state.is_escaping:
        if test_mode:
            log_event(px, estado, "[SIM] SCAPE BLOQUEADO")
            return True

        scape_danger(px, robot_state, SLOW_SPEED)
        px.last_cmd = "SCAPE"
        return True

    # ------------------------------------------------------------
    # 1. MODO SIMULADO
    # ------------------------------------------------------------
    if test_mode:
        return _execute_motion_sim(px, estado, cmd)

    # ------------------------------------------------------------
    # 2. EVITAR COMANDOS REDUNDANTES
    # ------------------------------------------------------------
    if px.last_cmd == cmd:
        return False

    # ------------------------------------------------------------
    # 3. VALIDACIÓN POR ESTADO
    # ------------------------------------------------------------
    if not _cmd_allowed_in_state(cmd, estado):
        log_event(px, estado, f"Comando bloqueado en {estado.name}: {cmd.name}")
        return False

    # ------------------------------------------------------------
    # 4. EJECUCIÓN REAL
    # ------------------------------------------------------------
    try:
        if cmd == Cmd.STOP:
            stop(px)

        elif cmd == Cmd.FORWARD:
            forward(px, FAST_SPEED)

        elif cmd == Cmd.FORWARD_SLOW:
            forward_slow(px, SLOW_SPEED)

        elif cmd == Cmd.BACKWARD:
            backward(px, SLOW_SPEED)

        elif cmd == Cmd.WHEELS_TURN_LEFT:
            turn_left(px)

        elif cmd == Cmd.WHEELS_TURN_RIGHT:
            turn_right(px)

        # Cámara
        elif cmd == Cmd.CAM_PAN_LEFT:
            pan_left(px)

        elif cmd == Cmd.CAM_PAN_RIGHT:
            pan_right(px)

        elif cmd == Cmd.CAM_TILT_TOP:
            tilt_top(px)

        elif cmd == Cmd.CAM_TILT_BOTTOM:
            tilt_bottom(px)

        elif cmd == Cmd.KEEP_ALIVE:
            pass

        else:
            log_event(px, Estado.ERR, f"Comando desconocido: {cmd}")
            stop(px)
            return False

        px.last_cmd = cmd
        log_event(px, estado, f"Ejecutado: {cmd.name}")
        return True

    except Exception as e:
        log_event(px, Estado.ERR, f"Error ejecutando {cmd}: {e}")
        stop(px)
        return False

def _cmd_allowed_in_state(cmd, estado):

    allowed = {
        Estado.SEARCH: {
            Cmd.CAM_PAN_LEFT, Cmd.CAM_PAN_RIGHT,
            Cmd.FORWARD_SLOW, Cmd.WHEELS_TURN_LEFT, Cmd.WHEELS_TURN_RIGHT,
            Cmd.STOP
        },

        Estado.RECENTER: {
            Cmd.CAM_PAN_LEFT, Cmd.CAM_PAN_RIGHT,
            Cmd.STOP
        },

        Estado.TRACK: {
            Cmd.FORWARD_SLOW, Cmd.WHEELS_TURN_LEFT, Cmd.WHEELS_TURN_RIGHT,
            Cmd.STOP
        },

        Estado.NEAR: {
            Cmd.BACKWARD, Cmd.STOP
        },

        Estado.IDLE: {Cmd.STOP},
        Estado.RESET: {Cmd.STOP},
    }

    return cmd in allowed.get(estado, {Cmd.STOP})

def _execute_motion_sim(px, estado, cmd):
    """
    En modo SIM:
    - Se ejecutan SOLO comandos de cámara y STOP.
    - Todo movimiento físico (ruedas) se bloquea.
    - La FSM sigue funcionando normalmente.
    """
    try:
        if cmd == Cmd.STOP:
            # No llamamos a px.stop() para evitar movimiento real
            log_event(px, estado, "[SIM] STOP")
            return True

        # Cámara permitida
        if cmd == Cmd.CAM_PAN_LEFT:
            pan_left(px)
        elif cmd == Cmd.CAM_PAN_RIGHT:
            pan_right(px)
        elif cmd == Cmd.CAM_TILT_TOP:
            tilt_top(px)
        elif cmd == Cmd.CAM_TILT_BOTTOM:
            tilt_bottom(px)
        else:
            # TODO lo demás se bloquea
            log_event(px, estado, f"[SIM] BLOQUEADO: {cmd.name}")
            return True

        log_event(px, estado, f"[SIM] Ejecutado: {cmd.name}")
        return True

    except Exception as e:
        log_event(px, estado, f"[SIM] Error ejecutando {cmd}: {e}")
        return False


# ============================================================
# SEGURIDAD
# ============================================================
def update_safety(px):
    raw = px.ultrasonic.read()

    if raw <= 0 or raw >= 400:
        return 999  # sin obstáculo

    return round(raw, 2)


def apply_safety(px, estado, accion, state):

    d = update_safety(px)

    # ------------------------------------------------------------
    # SIM: SCAPE lógico pero sin movimiento
    # ------------------------------------------------------------
    if px.test_mode:
        if d < DANGER_DISTANCE:
            log_event(px, estado, "[SIM] SCAPE (lógico) → SEARCH")
            return Estado.SEARCH, Cmd.STOP
        return estado, accion

    # ------------------------------------------------------------
    # 1. PELIGRO — activar SCAPE
    # ------------------------------------------------------------
    if d < DANGER_DISTANCE or state.is_escaping:

        terminado = scape_danger(px, state, SLOW_SPEED)

        if not terminado:
            return Estado.SCAPE, Cmd.KEEP_ALIVE

        state.search_lost_frames = 0
        state.search_found_frames = 0
        state.search_edge_frames = 0

        return Estado.SEARCH, Cmd.STOP

    # ------------------------------------------------------------
    # 2. ZONA DE PRECAUCIÓN
    # ------------------------------------------------------------
    if d < WARNING_DISTANCE and accion == Cmd.FORWARD:
        return estado, Cmd.FORWARD_SLOW

    return estado, accion


# ============================================================
# FUNCIONES
# ============================================================

def get_detection(px):
    params = Vilib.detect_obj_parameter

    raw = {
        "x": params.get("color_x", -1),
        "y": params.get("color_y", -1),
        "w": params.get("color_w", 0),
        "h": params.get("color_h", 0),
        "n": params.get("color_n", 0),
    }

    det = Det(
        n = raw["n"],
        x = raw["x"],
        y = raw["y"],
        w = raw["w"],
        h = raw["h"]
    )

    # ------------------------------------------------------------
    # FILTRO ANTI-FANTASMA
    # ------------------------------------------------------------
    # Fantasma típico: n>=1 pero w/h = 0
    if det.n >= 1 and (det.w == 0 or det.h == 0):
        det.n = 0

    # Fantasma por área incoherente
    if det.area < 50:
        det.n = 0

    # ------------------------------------------------------------
    # LOG SOLO CUANDO APARECE UNA DETECCIÓN REAL
    # ------------------------------------------------------------
    if det.valid_for_search and not px.last_det:
        log_det(px, px.estado_actual, det, raw, prefix="NEW DET → ")

    # Guardar última detección válida
    px.last_det = det if det.valid_for_search else None
    px.last_error_x = det.error_x
    px.last_error_y = det.error_y
    px.last_area = det.area

    return det, raw


def log_det(px, estado, det, raw, prefix=""):
    msg = (
        f"{prefix}"
        f"search={det.valid_for_search} "
        f"track={det.valid_for_track} "
        f"near={det.valid_for_near} "
        f"centered={det.is_centered} "
        f"n={raw['n']} w={raw['w']} h={raw['h']} "
        f"area={det.area} x={raw['x']} y={raw['y']} "
        f"err_x={det.error_x} err_y={det.error_y}"
    )
    log_event(px, estado, msg)


def do_yes(px, robot_state):
    current_time = time.time()

    if current_time < robot_state.yes_next_time:
        return False

    secuencia = [TILT_MAX, TILT_MIN, 0, TILT_MAX, TILT_MIN, 0]

    if robot_state.yes_step < len(secuencia):
        angulo = secuencia[robot_state.yes_step]
        px.set_cam_tilt_angle(angulo)

        robot_state.yes_next_time = current_time + 0.15
        robot_state.yes_step += 1

        if robot_state.yes_step == 1:
            log_event(px, px.estado_actual, "Iniciando gesto 'SI'")

        return False

    # Animación terminada
    robot_state.yes_step = 0
    robot_state.yes_next_time = 0
    return True


def print_dashboard(px, estado, accion, dist, state):
    os.system('clear')
    print("="*45)
    print(f" 🐾 PICAR-X DASHBOARD | Estado: {estado.name}")
    print("="*45)
    print(f" MOVIMIENTO: {accion.name}")
    print(f" DISTANCIA:  {dist} cm " + ("⚠️ DANGER" if dist < DANGER_DISTANCE else "SAFE"))
    print("-"*45)
    print(f" SERVO DIR:  {px.dir_current_angle:>5.1f}°")
    print(f" CAM PAN:    {px.last_pan:>5.1f}°")
    print(f" AREA:       {px.last_area}")
    print(f" ERR_X:      {px.last_error_x}")
    print(f" ESCAPANDO:  {state.is_escaping}")
    print("="*45)


# ============================================================
# ESTADOS
# ============================================================

def state_idle(px):
    log_event(px, Estado.IDLE, "Entrando en IDLE")
    return Estado.RESET, Cmd.STOP

def state_reset(px):
    log_event(px, Estado.RESET, "Entrando en RESET")

    # ------------------------------------------------------------
    # CENTRAR HARDWARE
    # ------------------------------------------------------------
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    px.set_dir_servo_angle(0)

    # ------------------------------------------------------------
    # SINCRONIZAR FLAGS FÍSICOS
    # ------------------------------------------------------------
    px.last_pan = 0
    px.last_tilt = 0
    px.dir_current_angle = 0

    # ------------------------------------------------------------
    # LIMPIAR FLAGS DE DETECCIÓN
    # ------------------------------------------------------------
    px.last_det = None
    px.last_error_x = 0
    px.last_error_y = 0
    px.last_area = 0

    # ------------------------------------------------------------
    # LIMPIAR ÚLTIMO COMANDO
    # ------------------------------------------------------------
    px.last_cmd = None

    # ------------------------------------------------------------
    # PASAR A SEARCH
    # ------------------------------------------------------------
    return Estado.SEARCH, Cmd.STOP


def state_search(px, estado, accion, st):
    det, raw = get_detection(px)

    # ============================================================
    # 1. ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, "Entrando en SEARCH")

        st.search_lost_frames = 0
        st.search_found_frames = 0
        st.search_edge_frames = 0
        st.search_cam_dir = 1
        st.search_wheels_dir = 1

        px.set_cam_pan_angle(0)
        px.last_pan = 0

        return Estado.SEARCH, Cmd.STOP

    # ============================================================
    # 2. RECUPERACIÓN TRAS SCAPE
    # ============================================================
    if st.last_sec_active:
        log_event(px, Estado.SEARCH, "[SEC] Recuperación tras SCAPE")

        px.set_cam_pan_angle(0)
        px.last_pan = 0

        st.search_lost_frames = 0
        st.search_found_frames = 0
        st.search_edge_frames = 0

        st.last_sec_active = False
        return Estado.SEARCH, Cmd.CAM_PAN_RIGHT

    # ============================================================
    # 3. DETECCIÓN VÁLIDA
    # ============================================================
    if det.valid_for_search:
        st.search_lost_frames = 0
        st.search_found_frames += 1

        # -------------------------
        # ZONA A — CENTRADO
        # -------------------------
        if det.valid_for_track and det.is_centered:
            if st.search_found_frames >= 3:
                return Estado.RECENTER, Cmd.STOP
            return Estado.SEARCH, Cmd.STOP

        # -------------------------
        # ZONA B — LATERAL
        # -------------------------
        if 40 < det.x < 600:
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT

        # -------------------------
        # ZONA C — BORDE
        # -------------------------
        st.search_edge_frames += 1

        # Si PAN puede moverse → PAN
        if (det.x < 40 and px.last_pan > PAN_MIN) or (det.x > 600 and px.last_pan < PAN_MAX):
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT

        # PAN en límite → GIRO DE CHASIS
        log_event(px, Estado.SEARCH, "Borde + PAN límite → GIRO DE CHASIS")
        st.search_wheels_dir = 1 if det.x > 320 else -1
        return Estado.SEARCH, Cmd.WHEELS_TURN_RIGHT if st.search_wheels_dir == 1 else Cmd.WHEELS_TURN_LEFT

    # ============================================================
    # 4. SIN DETECCIÓN — ZONA D
    # ============================================================
    st.search_found_frames = 0
    st.search_lost_frames += 1

    # Barrido PAN
    if px.last_pan >= PAN_MAX:
        st.search_cam_dir = -1
    elif px.last_pan <= PAN_MIN:
        st.search_cam_dir = 1

    # Si llevamos mucho sin ver nada → GIRO DE CHASIS
    if st.search_lost_frames > 60:
        log_event(px, Estado.SEARCH, "Perdido → GIRO DE CHASIS")
        return Estado.SEARCH, Cmd.WHEELS_TURN_RIGHT if st.search_wheels_dir == 1 else Cmd.WHEELS_TURN_LEFT

    # PAN normal
    return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if st.search_cam_dir == 1 else Cmd.CAM_PAN_LEFT


def state_recenter(px, estado, accion, st):
    det, raw = get_detection(px)

    # ============================================================
    # 1. ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.RECENTER:
        log_event(px, Estado.RECENTER, "Entrando en RECENTER")

        st.recenter_centered_frames = 0
        st.recenter_lost_frames = 0

        # El chasis SIEMPRE centrado en RECENTER
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        return Estado.RECENTER, Cmd.STOP

    # ============================================================
    # 2. SIN DETECCIÓN → tolerancia 5 frames
    # ============================================================
    if not det.valid_for_search:
        st.recenter_lost_frames += 1

        if st.recenter_lost_frames >= 5:
            log_event(px, Estado.RECENTER, "Sin detección → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        return Estado.RECENTER, Cmd.STOP

    # Reset de pérdida
    st.recenter_lost_frames = 0

    # ============================================================
    # 3. ERROR GRANDE → corregir PAN
    # ============================================================
    if abs(det.error_x) > 30:
        st.recenter_centered_frames = 0

        # PAN en límite y error grande → RECENTER no puede resolverlo
        if px.last_pan in (PAN_MAX, PAN_MIN) and abs(det.error_x) > 60:
            log_event(px, Estado.RECENTER, "PAN límite + error grande → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        # Corrección normal
        return Estado.RECENTER, (
            Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT
        )

    # ============================================================
    # 4. PAN en límite pero error pequeño → TRACK
    # ============================================================
    if px.last_pan in (PAN_MAX, PAN_MIN):
        log_event(px, Estado.RECENTER, "PAN límite + centrado → TRACK")

        px.set_cam_pan_angle(0)
        px.last_pan = 0
        st.just_recentered = time.time()

        return Estado.TRACK, Cmd.FORWARD_SLOW

    # ============================================================
    # 5. CENTRADO NORMAL
    # ============================================================
    st.recenter_centered_frames += 1

    if st.recenter_centered_frames >= 3:
        log_event(px, Estado.RECENTER, "Centrado ✔ → TRACK")

        px.set_cam_pan_angle(0)
        px.last_pan = 0
        st.just_recentered = time.time()

        return Estado.TRACK, Cmd.FORWARD_SLOW

    return Estado.RECENTER, Cmd.STOP


def state_track(px, estado, accion, st):
    det, raw = get_detection(px)

    # ============================================================
    # 1. ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.TRACK:
        log_event(px, Estado.TRACK, "Entrando en TRACK")

        # Servo SIEMPRE centrado al entrar
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        st.track_lost_frames = 0
        st.near_enter_frames = 0

        return Estado.TRACK, Cmd.FORWARD_SLOW

    # ============================================================
    # 2. COOLDOWN TRAS RECENTER
    # ============================================================
    if st.just_recentered:
        if time.time() - st.just_recentered < 0.3:
            return Estado.TRACK, Cmd.FORWARD_SLOW
        st.just_recentered = None

    # ============================================================
    # 3. PÉRDIDA DE BALIZA → SEARCH
    # ============================================================
    if not det.valid_for_track:
        st.track_lost_frames += 1

        if st.track_lost_frames >= 3:
            log_event(px, Estado.TRACK, "Perdida baliza → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        return Estado.TRACK, Cmd.FORWARD_SLOW

    # Reset de pérdida
    st.track_lost_frames = 0

    # ============================================================
    # 4. ENTRADA A NEAR
    # ============================================================
    if det.valid_for_near:
        st.near_enter_frames += 1

        if st.near_enter_frames >= 3:
            log_event(px, Estado.TRACK, "NEAR confirmado → NEAR")
            return Estado.NEAR, Cmd.STOP

        return Estado.TRACK, Cmd.FORWARD_SLOW

    st.near_enter_frames = 0

    # ============================================================
    # 5. CORRECCIÓN LATERAL PROPORCIONAL
    # ============================================================
    if abs(det.error_x) > 40:
        KP = 0.15
        target_angle = det.error_x * KP

        # Limitar ángulo
        target_angle = max(min(target_angle, SERVO_ANGLE_MAX), SERVO_ANGLE_MIN)

        # Solo mover si cambia
        if px.dir_current_angle != target_angle:
            px.set_dir_servo_angle(target_angle)
            px.dir_current_angle = target_angle

        return Estado.TRACK, Cmd.FORWARD_SLOW

    # ============================================================
    # 6. AVANCE RECTO (error pequeño)
    # ============================================================
    if px.dir_current_angle != 0:
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

    return Estado.TRACK, Cmd.FORWARD_SLOW


def state_near(px, estado, accion, st):
    det, raw = get_detection(px)

    # ============================================================
    # 1. ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.NEAR:
        log_event(px, Estado.NEAR, "Entrando en NEAR: frenado inmediato")

        px.stop()

        st.near_done_backward = False
        st.near_lost_frames = 0
        st.near_exit_frames = 0
        st.near_did_yes = False
        st.near_cooldown = None

        # Chasis siempre centrado
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        # Cámara estable
        px.set_cam_tilt_angle(0)
        px.last_tilt = 0

        return Estado.NEAR, Cmd.STOP

    # ============================================================
    # 2. VALIDACIÓN DE PRESENCIA
    # ============================================================
    if not det.valid_for_search:
        st.near_lost_frames += 1

        if st.near_lost_frames >= 5:
            log_event(px, Estado.NEAR, "Baliza perdida → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        return Estado.NEAR, Cmd.STOP

    st.near_lost_frames = 0

    # ============================================================
    # 3. RETROCESO DE CORTESÍA
    # ============================================================
    if not st.near_done_backward:
        log_event(px, Estado.NEAR, "Retroceso de cortesía")
        st.near_done_backward = True
        st.near_cooldown = time.time() + 0.4
        return Estado.NEAR, Cmd.BACKWARD

    # Esperar a que termine el backward
    if st.near_cooldown:
        if time.time() < st.near_cooldown:
            return Estado.NEAR, Cmd.KEEP_ALIVE
        else:
            px.stop()
            st.near_cooldown = None
            return Estado.NEAR, Cmd.STOP

    # ============================================================
    # 4. GESTO “SÍ”
    # ============================================================
    if not st.near_did_yes:
        terminado = do_yes(px, st)
        if terminado:
            log_event(px, Estado.NEAR, "Gesto 'sí' completado")
            st.near_did_yes = True
        return Estado.NEAR, Cmd.STOP

    # ============================================================
    # 5. MANTENIMIENTO / SALIDA
    # ============================================================
    if not det.valid_for_near:
        st.near_exit_frames += 1

        if st.near_exit_frames >= 10:
            log_event(px, Estado.NEAR, "Baliza alejada → TRACK")
            return Estado.TRACK, Cmd.STOP

        return Estado.NEAR, Cmd.STOP

    # Baliza sigue cerca → mantener
    st.near_exit_frames = 0
    return Estado.NEAR, Cmd.STOP


# ============================================================
# BUCLE PRINCIPAL
# ============================================================

def pet_mode(px, test_mode):
    print("🐾 Pet02 mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet02.log ===\n")

    hello_px(px)
    init_camera(px)
    init_flags(px)

    estado, accion = init_internal_state(px)
    state = RobotState()

    log_event(px, estado, "Inicio del sistema")

    while True:

        # ------------------------------------------------------------
        # 1. ACTUALIZAR ESTADO PREVIO
        # ------------------------------------------------------------
        px.last_state = px.estado_actual
        px.estado_actual = estado

        # ------------------------------------------------------------
        # 2. LECTURA DE SENSORES
        # ------------------------------------------------------------
        distancia_real = update_safety(px)

        # ------------------------------------------------------------
        # 3. SEGURIDAD (TIENE PRIORIDAD ABSOLUTA)
        # ------------------------------------------------------------
        escape_done = scape_danger(px, state)

        if state.is_escaping:
            # SCAPE controla el movimiento, no ejecutamos FSM
            execute_motion(px, estado, accion, state, test_mode)
            time.sleep(0.05)
            continue

        if escape_done:
            # SCAPE terminó → volvemos a SEARCH limpio
            estado = Estado.SEARCH
            accion = Cmd.STOP
            execute_motion(px, estado, accion, state, test_mode)
            time.sleep(0.05)
            continue

        # ------------------------------------------------------------
        # 4. FSM (solo si no hay SCAPE)
        # ------------------------------------------------------------
        if estado == Estado.IDLE:
            estado, accion = state_idle(px)

        elif estado == Estado.RESET:
            estado, accion = state_reset(px)

        elif estado == Estado.SEARCH:
            estado, accion = state_search(px, estado, accion, state)

        elif estado == Estado.RECENTER:
            estado, accion = state_recenter(px, estado, accion, state)

        elif estado == Estado.TRACK:
            estado, accion = state_track(px, estado, accion, state)

        elif estado == Estado.NEAR:
            estado, accion = state_near(px, estado, accion, state)

        # ------------------------------------------------------------
        # 5. EJECUTAR MOVIMIENTO (UNA SOLA VEZ)
        # ------------------------------------------------------------
        execute_motion(px, estado, accion, state, test_mode)

        # ------------------------------------------------------------
        # 6. DASHBOARD
        # ------------------------------------------------------------
        if not test_mode:
            print_dashboard(px, estado, accion, distancia_real, state)

        # ------------------------------------------------------------
        # 7. FRECUENCIA
        # ------------------------------------------------------------
        time.sleep(0.05)


# ============================================================
# ENTRYPOINT
# ============================================================

if __name__ == "__main__":
    import sys
    import os
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
