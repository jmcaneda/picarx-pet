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

FAST_SPEED = 35
SLOW_SPEED = 10
TURN_SPEED = 15

CAM_STEP = 4

SAFE_DISTANCE = 999
WARNING_DISTANCE = 35
DANGER_DISTANCE = 20

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
    WHEELS_ZIG_ZAG = 6
    BACKWARD = 7
    SCAPE = 8
    CAM_PAN_LEFT = 9
    CAM_PAN_RIGHT = 10
    CAM_TILT_TOP = 20
    CAM_TILT_BOTTOM = 30
    CAM_TILT_YES = 40
    CAM_PAN_NO = 50
    KEEP_ALIVE = 99 # Comando ficticio para mantener maniobra activa sin cambiar estado

class Det:
    """
    Detección fusionada: visión + distancia.
    """

    def __init__(self, n, x, y, w, h, distance, distance_raw, cx=CX, cy=CY):
        self.n = n
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.cx = cx
        self.cy = cy

        # Distancia
        self.distance = distance
        self.distance_raw = distance_raw

    # ------------------------------------------------------------
    # PROPIEDADES DE VISIÓN
    # ------------------------------------------------------------
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
        return abs(self.error_x) <= 40

    @property
    def valid_for_search(self):
        if self.n <= 0:
            return False
        if self.w <= 0 or self.h <= 0:
            return False
        if not (12 < self.w < 640 and 12 < self.h < 480):
            return False
        if not (300 < self.area < 240000):
            return False
        if not (0 < self.x < 640 and 0 < self.y < 480):
            return False
        return True

    @property
    def valid_for_near(self):
        if not self.valid_for_search:
            return False
        if self.area < 18000:
            return False
        if abs(self.error_x) > 40:
            return False
        if abs(self.error_y) > 120:
            return False
        return True

    # ------------------------------------------------------------
    # PROPIEDADES DE DISTANCIA
    # ------------------------------------------------------------
    @property
    def valid_distance(self):
        return 5 < self.distance < 350

    @property
    def near_by_distance(self):
        return self.distance < 35

    @property
    def too_close_to_measure(self):
        return (
            self.valid_for_search and
            self.area > 30000 and
            (self.distance > 80 or not self.valid_distance)
        )

    # ------------------------------------------------------------
    # FUSIÓN DE SENSORES
    # ------------------------------------------------------------
    @property
    def near_fused(self):
        """
        NEAR robusto:
        - visión manda cuando el sensor falla
        - si ambos coinciden → perfecto
        - si visión dice “muy cerca” y distancia falla → confiar en visión
        """
        if self.valid_for_near and self.near_by_distance:
            return True

        if self.valid_for_near and not self.valid_distance:
            return True

        if self.too_close_to_measure:
            return True

        return False


class RobotState:
    def __init__(self):

        # ============================================================
        # SEARCH — Exploración y adquisición de baliza
        # ============================================================

        # Número de frames consecutivos sin detección válida.
        # Si supera un umbral → giro de chasis. (*)
        self.search_lost_frames = 0          

        # Número de frames consecutivos con detección válida.
        # Se usa para histéresis antes de pasar a RECENTER. (*)
        self.search_found_frames = 0         

        # Dirección del barrido PAN: +1 derecha, -1 izquierda.
        # Se invierte al llegar a los límites del servo.
        self.search_cam_dir = 1              

        # Dirección del giro del chasis cuando SEARCH está perdido.
        # Se alterna para evitar bucles girando siempre al mismo lado.
        self.search_wheels_dir = 1           

        # Frames consecutivos en los que la baliza está en el borde.
        # Si supera un umbral → giro de chasis. (*)
        self.search_edge_frames = 0          


        # ============================================================
        # RECENTER — Alineación fina cuerpo/cámara
        # ============================================================

        # Frames consecutivos en los que la baliza está centrada.
        # Si supera un umbral → pasar a TRACK.
        self.recenter_centered_frames = 0    

        # Frames consecutivos sin detección durante RECENTER.
        # Si supera un umbral → volver a SEARCH.
        self.recenter_lost_frames = 0        

        # Marca temporal para evitar volver a RECENTER inmediatamente
        # después de haberlo completado (cooldown).
        self.just_recentered = None          


        # ============================================================
        # TRACK — Seguimiento dinámico de la baliza
        # ============================================================

        # Frames consecutivos sin detección válida durante TRACK.
        # Si supera un umbral → volver a SEARCH.
        self.track_lost_frames = 0           

        # Frames consecutivos centrado durante TRACK.
        # Se usa para suavizar movimientos y evitar oscilaciones.
        self.track_centered_frames = 0       


        # ============================================================
        # NEAR — Interacción cercana (frenado, retroceso, gesto)
        # ============================================================

        # Frames consecutivos en los que se cumplen condiciones de NEAR.
        # Evita entrar en NEAR por ruido.
        self.near_enter_frames = 0           

        # Frames consecutivos en los que se pierde la condición de NEAR.
        # Controla la salida suave de NEAR.
        self.near_exit_frames = 0            

        # Frames sin detección durante NEAR.
        # Si supera un umbral → volver a SEARCH.
        self.near_lost_frames = 0            

        # Indica si ya se ejecutó el retroceso de cortesía.
        self.near_done_backward = False      

        # Marca temporal para evitar reentradas rápidas en NEAR.
        self.near_cooldown = None            

        # Indica si ya se ejecutó el gesto "sí".
        self.near_did_yes = False            


        # Animación YES — control de pasos y temporización
        self.yes_step = 0
        self.yes_next_time = 0.0


        # ============================================================
        # SCAPE — Protocolos de seguridad y evasión
        # ============================================================

        # Indica si el robot está actualmente escapando.
        # Mientras sea True, la FSM queda bloqueada.
        self.is_escaping = False             

        # Tiempo en el que debe finalizar la maniobra de escape.
        self.escape_end_time = 0             

        # Indica si SEC ha estado activo recientemente.
        # SEARCH lo usa para reiniciar PAN y contadores.
        self.last_sec_active = False

        

# ============================================================
# INICIALIZACIÓN
# ============================================================

def init_camera(px):
    """
    Inicializa la cámara y el sistema de visión del robot.
    - Arranca la cámara física.
    - Configura la vista (sin invertir).
    - Activa la detección del color de la baliza.
    """

    # Iniciar cámara (sin volteos)
    Vilib.camera_start(vflip=False, hflip=False)

    # Mostrar por web (local=False evita errores si no hay GUI)
    Vilib.display(local=False, web=False)

    # Activar detección del color objetivo (baliza)
    Vilib.color_detect(BALIZA_COLOR)

    # Pequeña pausa para estabilizar la cámara
    time.sleep(0.5)


def init_internal_state(px):
    """
    Devuelve el estado inicial de la FSM y el primer comando.
    - Estado inicial: IDLE
    """
    return Estado.IDLE


def init_flags(px):
    """
    Inicializa todas las variables internas del robot.
    """

    # LOGGING
    px.last_log = None

    # ESTADOS
    px.last_state = None
    px.estado_actual = None
    px.last_cmd = "KEEP_ALIVE"

    # CÁMARA
    px.last_pan = 0
    px.last_tilt = 0

    # DIRECCIÓN DEL CHASIS
    px.dir_current_angle = 0

    # DETECCIÓN
    px.last_det = None

    # MOVIMIENTO
    px.changed_speed_slow = False
    px.forward_active = False

    # SEGURIDAD
    px.distance_real = SAFE_DISTANCE      # distancia filtrada
    px.distance_raw = SAFE_DISTANCE       # lectura cruda inicial
    px.ultra_valid = False                # opcional: útil para validadores


# ============================================================
# ACCIONES BÁSICAS
# ============================================================

def stop(px):
    if px.last_cmd == Cmd.STOP:
        return False

    px.stop()
    px.last_cmd = Cmd.STOP
    px.forward_active = False
    return True


def forward(px):
    cmd = Cmd.FORWARD_SLOW if px.changed_speed_slow else Cmd.FORWARD

    if px.last_cmd == cmd:
        return True

    if cmd == Cmd.FORWARD:
        px.forward(FAST_SPEED)
    else:
        px.forward(SLOW_SPEED)

    px.last_cmd = cmd
    px.forward_active = True
    return True


def forward_slow(px):
    if px.last_cmd == Cmd.FORWARD_SLOW:
        return False

    px.forward(SLOW_SPEED)
    px.last_cmd = Cmd.FORWARD_SLOW
    px.forward_active = True
    return True


def backward(px):
    # Bloqueo: no permitir backward directo después de forward
    if px.forward_active:
        log_event(px, "BACKWARD", "Bloqueado: veníamos de FORWARD sin STOP")
        return False

    if px.last_cmd == Cmd.BACKWARD:
        return False

    px.backward(SLOW_SPEED)
    px.last_cmd = Cmd.BACKWARD
    px.forward_active = False
    return True


# ------------------------------------------------------------
# GIRO CONTINUO REAL
# ------------------------------------------------------------

def turn_left(px):
    if px.last_cmd == Cmd.WHEELS_TURN_LEFT:
        return False

    px.set_dir_servo_angle(SERVO_ANGLE_MIN)
    px.dir_current_angle = SERVO_ANGLE_MIN

    px.last_cmd = Cmd.WHEELS_TURN_LEFT
    px.forward_active = False   # ← IMPORTANTE
    return True


def turn_right(px):
    if px.last_cmd == Cmd.WHEELS_TURN_RIGHT:
        return False

    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
    px.dir_current_angle = SERVO_ANGLE_MAX

    px.last_cmd = Cmd.WHEELS_TURN_RIGHT
    px.forward_active = False   # ← IMPORTANTE
    return True


def zig_zag(px):
    det, raw = get_detection(px)
    px.last_det = det

    servo_angle = round(clamp(det.error_x * 0.05, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX),1)

    px.set_dir_servo_angle(servo_angle)
    px.dir_current_angle = servo_angle

    px.last_cmd = Cmd.WHEELS_ZIG_ZAG
    px.forward_active = False   # ← IMPORTANTE
    return servo_angle


# ============================================================
# MOVIMIENTOS DE CÁMARA SEGUROS
# ============================================================

def pan_right(px, step=CAM_STEP):
    new_angle = px.last_pan + step

    if new_angle >= PAN_MAX:
        new_angle = PAN_MAX

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

    px.last_cmd = Cmd.CAM_PAN_RIGHT
    px.forward_active = False   # ← IMPORTANTE

    return 1 if new_angle < PAN_MAX else 0


def pan_left(px, step=CAM_STEP):
    new_angle = px.last_pan - step

    if new_angle <= PAN_MIN:
        new_angle = PAN_MIN

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)

    px.last_cmd = Cmd.CAM_PAN_LEFT
    px.forward_active = False   # ← IMPORTANTE

    return 1 if new_angle > PAN_MIN else 0


def tilt_top(px, step=CAM_STEP):
    new_angle = px.last_tilt + step

    if new_angle >= TILT_MAX:
        new_angle = TILT_MAX

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

    px.last_cmd = Cmd.CAM_TILT_TOP
    px.forward_active = False   # ← IMPORTANTE

    return 1 if new_angle < TILT_MAX else 0



def tilt_bottom(px, step=CAM_STEP):
    new_angle = px.last_tilt - step

    if new_angle <= TILT_MIN:
        new_angle = TILT_MIN

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

    px.last_cmd = Cmd.CAM_TILT_BOTTOM
    px.forward_active = False   # ← IMPORTANTE

    return 1 if new_angle > TILT_MIN else 0


def tilt_yes(px):

    if px.last_cmd == Cmd.CAM_TILT_YES:
        return False

    log_event(px, px.last_state, "Iniciando gesto 'SI'")

    secuencia = [TILT_MAX, TILT_MIN, 0, TILT_MAX, TILT_MIN, 0]

    for angulo in secuencia:
        px.set_cam_tilt_angle(angulo)
        time.sleep(0.15)

    px.last_cmd = Cmd.CAM_TILT_YES
    px.forward_active = False   # ← IMPORTANTE

    return True



# ============================================================
# SEGURIDAD
# ============================================================
def update_safety(px):
    """
    Lee el sensor ultrasónico UNA sola vez por ciclo,
    aplica filtro básico y actualiza px.distance_real.
    """

    raw = px.ultrasonic.read()

    # Filtro de valores basura
    if raw <= 0 or raw >= 400:
        d = 999
    else:
        d = round(raw, 2)

    # Guardar en el robot
    px.distance_real = d
    px.distance_raw = raw  # opcional, útil para logs

    return d



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
        h = raw["h"],
        distance = px.distance_real,      # filtrada
        distance_raw = px.distance_raw    # cruda
    )

    # Filtros anti-fantasma
    if det.x < 0 or det.y < 0:
        det.n = 0
    if det.n >= 1 and (det.w <= 0 or det.h <= 0):
        det.n = 0
    if det.area < 300:
        det.n = 0

    return det, raw


def log_event(px, estado, msg):
    # Si es igual al último mensaje, no lo repitas
    if px.last_log == (estado, msg):
        return

    px.last_log = (estado, msg)

    ts = time.strftime("%H:%M:%S", time.localtime())
    line = f"[{ts}] [{estado}] {msg}\n"

    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line)


def log_det(px, estado, det, raw, state, prefix=""):
    # Si state es None, usamos "N/A", si no, el valor real
    f_lost = state.recenter_lost_frames if state else "N/A"
    
    msg = (
        f"{prefix} f_lost={f_lost} "
        f"search={det.valid_for_search} "
        f"near={det.valid_for_near} "
        f"centered={det.is_centered} "
        f"n={raw['n']} w={raw['w']} h={raw['h']} "
        f"area={det.area} x={raw['x']} y={raw['y']} "
        f"err_x={det.error_x} err_y={det.error_y}"
    )
    log_event(px, estado, msg)


def print_dashboard(px, estado, st, dist, test_mode):
    os.system('clear')
    print("="*45)
    print(f" 🐾 PICAR-X DASHBOARD | Estado: {estado.name}")
    print(f" 🐾 TEST MODE: {test_mode}")
    print("="*45)

    # Movimiento reportado por el último estado
    print(f" MOVIMIENTO: {px.last_cmd}")

    # Seguridad
    print(f" DISTANCIA:  {dist} cm " + ("⚠️ DANGER" if dist < DANGER_DISTANCE else "SAFE"))
    print("-"*45)

    # Hardware
    print(f" SERVO DIR:  {px.dir_current_angle:>5.1f}°")
    print(f" CAM PAN:    {px.last_pan:>5.1f}°")
    print(f" CAM TILT:   {px.last_tilt:>5.1f}°")

    # Última detección
    print(f" AREA:       {px.last_det.area}")
    print(f" ERR_X:      {px.last_det.error_x}")

    print("="*45)
    print(" Presiona Ctrl+C para detener")
    

def clamp(value, min_value, max_value):
    """
    Limita 'value' entre min_value y max_value.
    """
    return max(min_value, min(value, max_value))


# ============================================================
# ESTADOS
# ============================================================

def state_idle(px, estado, state, distancia_real, test_mode):
    """
    Estado IDLE:
    - Estado inicial del sistema.
    - No realiza detección ni movimiento.
    - Solo ejecuta STOP y pasa a RESET.
    """
    det, raw = get_detection(px)
    px.last_det = det
    if px.last_state != Estado.IDLE:
        log_event(px, Estado.IDLE, f"Entrando en IDLE (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.IDLE, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")   

    stop(px)

    px.last_state = Estado.IDLE
    return Estado.RESET


def state_reset(px, estado, st, distancia_real, test_mode):
    """
    Estado RESET:
    - Centra todos los servos.
    - Limpia flags físicos.
    - Limpia contadores del RobotState.
    - Garantiza que el robot está quieto.
    - Transiciona inmediatamente a SEARCH.
    """
    det, raw = get_detection(px)
    px.last_det = det
    # Registrar entrada al estado solo una vez
    if px.last_state != Estado.RESET:
        log_event(px, Estado.RESET, f"Entrando en RESET (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.RESET, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")   

    # ------------------------------------------------------------
    # DETENER ROBOT
    # ------------------------------------------------------------
    stop(px)

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
    px.changed_speed_slow = False

    # ------------------------------------------------------------
    # LIMPIAR CONTADORES DEL ESTADO INTERNO
    # ------------------------------------------------------------
    st.search_lost_frames = 0
    st.search_found_frames = 0
    st.search_edge_frames = 0
    st.search_cam_dir = 1
    st.search_wheels_dir = 1

    st.recenter_centered_frames = 0
    st.recenter_lost_frames = 0
    st.just_recentered = None

    st.track_lost_frames = 0
    st.track_centered_frames = 0

    st.near_enter_frames = 0
    st.near_exit_frames = 0
    st.near_lost_frames = 0
    st.near_done_backward = False
    st.near_cooldown = None
    st.near_did_yes = False

    st.yes_step = 0
    st.yes_next_time = 0.0

    st.is_escaping = False
    st.escape_end_time = 0
    st.last_sec_active = False

    # ------------------------------------------------------------
    # ACTUALIZAR ESTADO
    # ------------------------------------------------------------
    px.last_state = Estado.RESET

    # ------------------------------------------------------------
    # PASAR A SEARCH
    # ------------------------------------------------------------
    return Estado.SEARCH


def state_search(px, estado, st, distancia_real, test_mode):
    det, raw = get_detection(px)
    px.last_det = det

    # ============================================================
    # ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, f"Entrando en SEARCH (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.SEARCH, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")   

        # Reset de contadores
        st.search_lost_frames = 0
        st.search_found_frames = 0
        st.search_edge_frames = 0
        st.search_cam_dir = 1
        st.search_wheels_dir = 1
        st.is_escaping = False

        # Reset de hardware
        px.set_cam_pan_angle(0)
        px.last_pan = 0
        px.set_cam_tilt_angle(0)
        px.last_tilt = 0
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0
        px.changed_speed_slow = False
        px.last_state = Estado.SEARCH
        px.last_cmd = Cmd.STOP
        stop(px)

        return Estado.SEARCH

    # ============================================================
    # SEGURIDAD SEARCH
    # ============================================================
    update_safety(px)

    if det.near_fused:
            log_event(px, Estado.SEARCH, f"Baliza muy cerca según fusión → NEAR")
            stop(px)
            px.last_cmd = Cmd.STOP
            return Estado.NEAR

    # 1. Peligro extremo → SCAPE inmediato
    if px.distance_real < DANGER_DISTANCE and not st.is_escaping:
        log_event(px, Estado.SEARCH, f"🚨 Peligro extremo distancia={px.distance_real} → SCAPE")
        stop(px)
        px.last_cmd = Cmd.STOP
        backward(px)
        px.last_cmd = Cmd.BACKWARD
        time.sleep(0.4)
        stop(px)
        px.last_cmd = Cmd.STOP

        st.is_escaping = True
        st.escape_end_time = time.time() + 1.0
        px.last_cmd = Cmd.SCAPE
        return Estado.SEARCH

    # 2. Advertencia → frenar inercia (solo si no estamos escapando)
    if px.distance_real < WARNING_DISTANCE and not st.is_escaping:
        if not px.changed_speed_slow:  # evita spam de logs
            log_event(px, Estado.SEARCH, f"⚠️ Advertencia: objeto a {px.distance_real} cm → frenado preventivo")
        stop(px)
        px.last_cmd = Cmd.STOP
        px.changed_speed_slow = True

    # 3. Salida del modo escape
    if st.is_escaping and time.time() >= st.escape_end_time:
        st.is_escaping = False
        px.changed_speed_slow = False


    # ============================================================
    # DETECCIÓN VÁLIDA
    # ============================================================
    if det.valid_for_search:
        st.search_lost_frames = 0
        st.search_found_frames += 1

        # Centrado estable → RECENTER
        if det.is_centered:
            if st.search_found_frames >= 3:
                log_event(px, Estado.SEARCH,
                          f"Centrado estable ({st.search_found_frames} frames) → RECENTER")
                px.last_state = Estado.SEARCH
                return Estado.RECENTER

        # Ajuste de PAN si error_x grande
        if abs(det.error_x) > 40:
            if det.error_x > 0:
                log_event(px, Estado.SEARCH, "Baliza a la derecha → PAN DERECHA")
                px.last_cmd = "PAN_RIGHT"
                if pan_right(px) == 0:
                    st.search_cam_dir *= -1
            else:
                log_event(px, Estado.SEARCH, "Baliza a la izquierda → PAN IZQUIERDA")
                px.last_cmd = "PAN_LEFT"
                if pan_left(px) == 0:
                    st.search_cam_dir *= -1

            px.last_state = Estado.SEARCH
            return Estado.SEARCH

    # ============================================================
    # SIN DETECCIÓN → barrido de cámara
    # ============================================================
    log_event(px, Estado.SEARCH, "SIN DETECCIÓN VÁLIDA → PANEO")

    # Barrido automático
    if st.search_cam_dir > 0:
        moved = pan_right(px)
    else:
        moved = pan_left(px)

    # Si llegó al límite, invertir dirección
    if moved == 0:
        st.search_cam_dir *= -1

    st.search_found_frames = 0
    st.search_lost_frames += 1
    st.search_edge_frames = 0

    px.last_cmd = Cmd.KEEP_ALIVE
    px.last_state = Estado.SEARCH
    return Estado.SEARCH


def state_recenter(px, estado, st, distancia_real,test_mode):
    det, raw = get_detection(px)
    px.last_det = det

    # ============================================================
    # ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.RECENTER:
        log_event(px, Estado.RECENTER, f"Entrando en RECENTER (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.RECENTER, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.") 
            return Estado.SEARCH  # en modo test, no entramos a RECENTER para evitar movimientos

        st.recenter_centered_frames = 0
        st.recenter_lost_frames = 0
        st.just_recentered = None

        px.last_state = Estado.RECENTER
        px.last_cmd = Cmd.STOP
        stop(px)

        return Estado.RECENTER

    
    # ============================================================
    # SIN DETECCIÓN → volver a SEARCH
    # ============================================================
    if not det.valid_for_search:
        st.recenter_lost_frames += 1
        if st.recenter_lost_frames >= 3:
            log_event(px, Estado.RECENTER, "Pérdida de detección → SEARCH")
            return Estado.SEARCH
        return Estado.RECENTER

    # ============================================================
    # DETECCIÓN VÁLIDA → alinear cuerpo
    # ============================================================
    st.recenter_lost_frames = 0

    # Si ya está centrado → TRACK
    if abs(det.error_x) < 20:
        st.recenter_centered_frames += 1
        if st.recenter_centered_frames >= 4:
            log_event(px, Estado.RECENTER, "Alineación completada → TRACK")
            px.last_cmd = "STOP"
            stop(px)
            return Estado.TRACK
        return Estado.RECENTER

    # ============================================================
    # CORRECCIÓN DE DIRECCIÓN
    # ============================================================
    log_event(px, Estado.RECENTER, f"Corrigiendo dirección, error_x={det.error_x}")
    error = det.error_x
    if abs(error) <= 20:
        # Centrado razonable → avanzar recto
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

    else:
        # Error significativo → zig-zag para corregir
        angulo = zig_zag(px)
        log_event(px, Estado.TRACK, f"Error significativo → zig-zag para corregir (ángulo={angulo})")

    # ============================================================
    # SEGURIDAD RECENTER
    # ============================================================
    update_safety(px)

    # Si la visión o la distancia dicen “muy cerca”, pasar a NEAR 
    if det.near_fused: 
        log_event(px, Estado.RECENTER, "Baliza muy cerca según fusión → NEAR") 
        stop(px)
        px.last_cmd = Cmd.STOP
        return Estado.NEAR

    # 1. Peligro extremo → SCAPE inmediato
    if px.distance_real < DANGER_DISTANCE and not st.is_escaping:
        log_event(px, Estado.RECENTER, f"🚨 Peligro extremo distancia={px.distance_real} → SCAPE")
        stop(px)
        px.last_cmd = Cmd.STOP
        backward(px)
        px.last_cmd = Cmd.BACKWARD
        time.sleep(0.4)
        stop(px)
        px.last_cmd = Cmd.STOP

        st.is_escaping = True
        st.escape_end_time = time.time() + 1.0
        px.last_cmd = Cmd.SCAPE
        return Estado.SEARCH   # ← IMPORTANTE: volver a SEARCH

    # 2. Advertencia → frenar y salir de RECENTER
    if px.distance_real < WARNING_DISTANCE and not st.is_escaping:
        log_event(px, Estado.RECENTER, f"⚠️ Advertencia: objeto a {px.distance_real} cm → frenado preventivo")
        stop(px)
        px.last_cmd = Cmd.STOP
        return Estado.SEARCH   # ← IMPORTANTE: no continuar RECENTER

    # 3. Salida del modo escape
    if st.is_escaping and time.time() >= st.escape_end_time:
        st.is_escaping = False


    # 3. Avance muy pequeño (solo para reposicionar)
    forward(px)
    time.sleep(0.03)   # mucho más pequeño que antes

    # 4. No resetear servo ni cámara
    # px.set_dir_servo_angle(0)  ← NO
    # px.set_cam_pan_angle(0)    ← NO

    return Estado.RECENTER



def state_track(px, estado, st, distancia_real,test_mode):
    det, raw = get_detection(px)
    px.last_det = det

    # ============================================================
    # ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.TRACK:
        log_event(px, Estado.TRACK, f"Entrando en TRACK (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.TRACK, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")
            return Estado.SEARCH  # en modo test, no entramos a TRACK para evitar movimientos

        st.track_lost_frames = 0
        st.track_centered_frames = 0

        px.last_state = Estado.TRACK
        px.last_cmd = Cmd.STOP
        stop(px)

        return Estado.TRACK

    
    # ============================================================
    # SIN DETECCIÓN → volver a SEARCH
    # ============================================================
    if not det.valid_for_search:
        st.track_lost_frames += 1
        if st.track_lost_frames >= 3:
            log_event(px, Estado.TRACK, "Pérdida de detección → SEARCH")
            return Estado.SEARCH
        return Estado.TRACK

    # Detección válida
    st.track_lost_frames = 0

    # ============================================================
    # NEAR → si estamos realmente cerca
    # ============================================================
    if det.valid_for_near:
        log_event(px, Estado.TRACK, "Baliza muy cerca → NEAR")
        stop(px)
        px.last_cmd = Cmd.STOP
        return Estado.NEAR

    # ============================================================
    # CORRECCIÓN DE DIRECCIÓN
    # ============================================================
    log_event(px, Estado.TRACK, f"Corrigiendo dirección, error_x={det.error_x}")
    error = det.error_x
    if abs(error) <= 20:
        # Centrado razonable → avanzar recto
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

    else:
        # Error significativo → zig-zag para corregir
        angulo = zig_zag(px)
        log_event(px, Estado.TRACK, f"Error significativo → zig-zag para corregir (ángulo={angulo})")
    
    # ============================================================
    # SEGURIDAD TRACK
    # ============================================================
    update_safety(px)

    if det.near_fused:
        stop(px)
        px.last_cmd = Cmd.STOP
        return Estado.NEAR

    # 1. Peligro extremo → SCAPE inmediato
    if px.distance_real < DANGER_DISTANCE and not st.is_escaping:
        log_event(px, Estado.TRACK, f"🚨 Peligro extremo distancia={px.distance_real} → SCAPE")
        stop(px)
        px.last_cmd = Cmd.STOP
        backward(px)
        px.last_cmd = Cmd.BACKWARD
        time.sleep(0.4)
        stop(px)
        px.last_cmd = Cmd.STOP

        st.is_escaping = True
        st.escape_end_time = time.time() + 1.0
        px.last_cmd = Cmd.SCAPE
        return Estado.SEARCH   # ← IMPORTANTE: volver a SEARCH

    # 2. Advertencia → reducir velocidad y evitar empuje
    if px.distance_real < WARNING_DISTANCE and not st.is_escaping:
        if not px.changed_speed_slow:  # evita spam de logs
            log_event(px, Estado.TRACK, f"⚠️ Advertencia: objeto a {px.distance_real} cm → velocidad reducida")
        px.changed_speed_slow = True   # TRACK sí usa forward(), así que esto es útil

    # 3. Salida del modo escape
    if st.is_escaping and time.time() >= st.escape_end_time:
        st.is_escaping = False
        px.changed_speed_slow = False


    forward(px)
    return Estado.TRACK


def state_near(px, estado, st, distancia_real, test_mode):
    det, raw = get_detection(px)
    px.last_det = det

    # ============================================================
    # ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.NEAR:
        log_event(px, Estado.NEAR, f"Entrando en NEAR (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.NEAR, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")
            return Estado.SEARCH

        st.near_enter_frames = 0
        st.near_exit_frames = 0
        st.near_lost_frames = 0
        st.near_done_backward = False
        st.near_did_yes = False
        st.near_cooldown = time.time() + 1.0

        st.yes_step = 0
        st.yes_next_time = 0.0

        stop(px)
        px.last_cmd = "STOP"
        px.last_state = Estado.NEAR

        return Estado.NEAR

    # ============================================================
    # SEGURIDAD NEAR (fusión)
    # ============================================================
    update_safety(px)

    # 1. Peligro extremo → SCAPE inmediato
    if px.distance_real < DANGER_DISTANCE and not st.is_escaping:
        log_event(px, Estado.NEAR, f"🚨 Peligro extremo distancia={px.distance_real} → SCAPE")
        stop(px)
        backward(px)
        time.sleep(0.4)
        stop(px)

        st.is_escaping = True
        st.escape_end_time = time.time() + 1.0
        px.last_cmd = Cmd.SCAPE
        return Estado.SEARCH

    # 2. Advertencia → detener todo movimiento
    if px.distance_real < WARNING_DISTANCE and not st.is_escaping:
        log_event(px, Estado.NEAR, f"⚠️ Advertencia: objeto a {px.distance_real} cm → frenado total")
        stop(px)
        return Estado.NEAR

    # 3. Salida del modo escape
    if st.is_escaping and time.time() >= st.escape_end_time:
        st.is_escaping = False

    # ============================================================
    # NUEVO: si la visión dice que sigue muy cerca → mantener NEAR
    # ============================================================
    if det.near_fused:
        # Esto evita que el robot salga de NEAR por ruido del sensor
        st.near_lost_frames = 0
        return Estado.NEAR

    # ============================================================
    # SIN DETECCIÓN → salir de NEAR
    # ============================================================
    if not det.valid_for_search:
        st.near_lost_frames += 1
        if st.near_lost_frames >= 3:
            log_event(px, Estado.NEAR, "Pérdida de detección → SEARCH")
            return Estado.SEARCH
        return Estado.NEAR

    # ============================================================
    # RETROCESO (solo una vez)
    # ============================================================
    if not st.near_done_backward:
        log_event(px, Estado.NEAR, "Retroceso de cortesía")
        backward(px)
        time.sleep(0.25)
        stop(px)
        st.near_done_backward = True
        return Estado.NEAR

    # ============================================================
    # GESTO “SÍ” (solo una vez)
    # ============================================================
    if not st.near_did_yes:
        log_event(px, Estado.NEAR, "Ejecutando gesto 'SÍ'")
        tilt_yes(px)
        st.near_did_yes = True
        return Estado.NEAR

    # ============================================================
    # SALIDA DE NEAR (cooldown terminado)
    # ============================================================
    if time.time() >= st.near_cooldown:
        log_event(px, Estado.NEAR, "Fin de interacción → SEARCH")
        stop(px)
        px.last_cmd = "STOP"
        return Estado.SEARCH

    return Estado.NEAR


# ============================================================
# BUCLE PRINCIPAL
# ============================================================

def pet_mode(px, test_mode):
    print("🐾 Pet02 mode")

    with open(LOG_PATH, "w", encoding="utf-8") as f:
        f.write("=== Start of pet02.log ===\n")

    # music.music_set_volume(20)
    # sound_path = os.path.join(SOUNDS_DIR, "sounds_angry.wav")
    # music.sound_play(sound_path)
    #time.sleep(0.05)

    hello_px(px)
    init_camera(px)
    init_flags(px)
    estado = init_internal_state(px)
    check_robot(px,log_event)
    state = RobotState()
    log_event(px, estado, "Inicio del sistema")
    ciclo_dashboard = 0
    test = test_mode

    while True:

        update_safety(px)


        if estado == Estado.IDLE:
            estado = state_idle(px, estado, state, px.distance_real, test)

        elif estado == Estado.RESET:
            estado = state_reset(px, estado, state, px.distance_real, test)

        elif estado == Estado.SEARCH:
            estado = state_search(px, estado, state, px.distance_real, test)

        elif estado == Estado.RECENTER:
            estado = state_recenter(px, estado, state, px.distance_real, test)

        elif estado == Estado.TRACK:
            estado = state_track(px, estado, state, px.distance_real, test)

        elif estado == Estado.NEAR:
            estado = state_near(px, estado, state, px.distance_real, test)

        ciclo_dashboard += 1
        if not test_mode and ciclo_dashboard % 5 == 0:
            print_dashboard(px, estado, state, px.distance_real, test)

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
