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
MAX_TRACK_ANGLE = 12

CX = 320
CY = 240

FAST_SPEED = 35
SLOW_SPEED = 10
TURN_SPEED = 15

CAM_STEP = 4

SAFE_DISTANCE = 999
WARNING_DISTANCE = 35
DANGER_DISTANCE = 10

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
    WHEELS_ANGLE_ERROR_X = 6
    WHEELS_ANGLE_PAN = 7
    BACKWARD = 8
    SCAPE = 9
    CAM_PAN_LEFT = 10
    CAM_PAN_RIGHT = 15
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
    def is_proportional(self):
        ratio = self.w / self.h if self.h != 0 else 0
        return 0.3 < ratio < 3.5 # Evita detectar líneas largas o reflejos extraños

    @property
    def valid_for_search(self):
        # Debe haber detección
        if self.n < 1:
            return False

        # Evitar formas absurdas (líneas, reflejos)
        if not self.is_proportional:
            return False

        # Tamaño mínimo razonable
        if self.w < 8 or self.h < 8:
            return False

        # Área mínima para evitar ruido
        if self.area < 800:
            return False

        # Permitir detecciones en el borde del frame
        if not (0 <= self.x <= 640 and 0 <= self.y <= 480):
            return False

        return True


    @property
    def valid_for_near(self):
        if not self.valid_for_search: # Si ni siquiera la veo bien, no puede estar cerca
            return False
        if self.area < 35000: # Pero si la veo, tiene que ser grande (está cerca)
            return False
        if abs(self.error_x) > 35: # <--- Más estricto que los 40 de RECENTER
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
            self.area > 45000 and
            not self.valid_distance
        )


    # ------------------------------------------------------------
    # FUSIÓN DE SENSORES
    # ------------------------------------------------------------
    @property
    def near_fused(self):
        # visión + distancia coherente para NEAR
        if self.valid_for_near and self.near_by_distance:
            return True

        if self.valid_for_near and not self.valid_distance and self.area > 45000:
            return True

        if self.too_close_to_measure:
            return True

        return False


class RobotState:
    def __init__(self):

        # ============================================================
        # SEARCH — Exploración y adquisición de baliza
        # ============================================================

        self.search_lost_frames = 0          # Frames sin detección válida
        self.search_found_frames = 0         # Frames con detección válida
        self.search_centered_frames = 0
        self.search_cam_dir = 1              # Dirección del barrido PAN
        self.search_wheels_dir = 1           # Dirección del giro del chasis
        self.search_edge_frames = 0          # Frames con baliza en el borde
        self.lost_in_space = False           # Señal de pérdida total


        # ============================================================
        # RECENTER — Alineación fina cuerpo/cámara
        # ============================================================

        self.recenter_centered_frames = 0    # Frames centrado
        self.recenter_lost_frames = 0        # Frames sin detección
        self.just_recentered = None          # Cooldown tras RECENTER


        # ============================================================
        # TRACK — Seguimiento dinámico de la baliza
        # ============================================================

        self.track_lost_frames = 0           # Frames sin detección
        self.track_centered_frames = 0       # Frames centrado (suavizado)


        # ============================================================
        # NEAR — Interacción cercana (frenado, retroceso, gesto)
        # ============================================================

        # Frames consecutivos en los que la baliza está "muy cerca".
        # Evita entrar en NEAR por ruido.
        self.near_hold_frames = 0

        # Frames consecutivos en los que la condición de NEAR deja de cumplirse.
        # Controla la salida suave de NEAR.
        self.near_exit_frames = 0

        # Frames sin detección durante NEAR.
        # Si supera un umbral → volver a SEARCH.
        self.near_lost_frames = 0

        # Indica si ya se ejecutó el retroceso de cortesía.
        self.near_backed = False

        # Indica si ya se ejecutó el gesto "sí".
        self.near_nodded = False

        # Cooldown para evitar reentradas rápidas en NEAR.
        self.near_cooldown = 0

        # Control interno para animación YES (si lo usas en el futuro)
        self.yes_step = 0
        self.yes_next_time = 0.0


        # ============================================================
        # SCAPE — Protocolos de seguridad y evasión
        # ============================================================

        self.is_escaping = False             # Señal de escape activo
        self.escape_end_time = 0             # Tiempo de fin de escape
        self.last_sec_active = False         # Señal de seguridad reciente


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


# ============================================================
# ACCIONES BÁSICAS
# ============================================================

def stop(px):

    px.stop()
    px.last_cmd = Cmd.STOP
    px.forward_active = False
    return True

def forward(px):
    # Bloqueo: no permitir forward directo después de backward
    if px.last_cmd == Cmd.BACKWARD:
        log_event(px, px.last_state, "Bloqueado: veníamos de BACKWARD sin STOP")
        return False

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


def backward(px):
    # Bloqueo: no permitir backward directo después de forward
    if px.forward_active:
        log_event(px, px.last_state, "Bloqueado: veníamos de FORWARD sin STOP")
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
    
    return True


def turn_right(px):
    if px.last_cmd == Cmd.WHEELS_TURN_RIGHT:
        return False

    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
    px.dir_current_angle = SERVO_ANGLE_MAX

    px.last_cmd = Cmd.WHEELS_TURN_RIGHT
    
    return True


def wheels_angle_error_x(px, det):

    servo_angle = round(clamp(det.error_x * 0.4, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX),1)

    px.set_dir_servo_angle(servo_angle)
    px.dir_current_angle = servo_angle

    px.last_cmd = Cmd.WHEELS_ANGLE_ERROR_X
    
    return servo_angle

def wheels_angle_pan(px, det):

    servo_angle = round(clamp(px.last_pan, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX),1)

    px.set_dir_servo_angle(servo_angle)
    px.dir_current_angle = servo_angle

    px.last_cmd = Cmd.WHEELS_ANGLE_PAN
    
    return servo_angle


def rock_robot(px):
    stop(px)
    time.sleep(0.05)
    backward(px)
    time.sleep(0.05)
    stop(px)
    px.changed_speed_slow = True
    forward(px)
    time.sleep(0.05)
    stop(px)

    return True

def circle_robot(px, direction=1):
    step = CAM_STEP
    step *= 2
    if direction > 0:
        turn_right(px)
        pan_left(px, step)
    else:
        turn_left(px)
        pan_right(px, step)

    px.changed_speed_slow = False
    forward(px)
    time.sleep(1)
    stop(px)

    return True


# ============================================================
# MOVIMIENTOS DE CÁMARA SEGUROS
# ============================================================

def pan_right(px, step=CAM_STEP):
    new_angle = px.last_pan + step
    alcanzado_limite = 1

    if new_angle >= PAN_MAX:
        new_angle = PAN_MAX
        alcanzado_limite = 0 # Avisamos que no se puede mover más a la derecha

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)
    px.last_cmd = Cmd.CAM_PAN_RIGHT

    return alcanzado_limite

def pan_left(px, step=CAM_STEP):
    new_angle = px.last_pan - step
    alcanzado_limite = 1

    if new_angle <= PAN_MIN:
        new_angle = PAN_MIN
        alcanzado_limite = 0 # Avisamos que no se puede mover más a la izquierda

    px.last_pan = new_angle
    px.set_cam_pan_angle(px.last_pan)
    px.last_cmd = Cmd.CAM_PAN_LEFT

    return alcanzado_limite


def tilt_top(px, step=CAM_STEP):
    new_angle = px.last_tilt + step

    if new_angle >= TILT_MAX:
        new_angle = TILT_MAX

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

    px.last_cmd = Cmd.CAM_TILT_TOP

    return 1 if new_angle <= TILT_MAX else 0


def tilt_bottom(px, step=CAM_STEP):
    new_angle = px.last_tilt - step

    if new_angle <= TILT_MIN:
        new_angle = TILT_MIN

    px.last_tilt = new_angle
    px.set_cam_tilt_angle(px.last_tilt)

    px.last_cmd = Cmd.CAM_TILT_BOTTOM

    return 1 if new_angle >= TILT_MIN else 0


def tilt_yes(px):
    if px.last_cmd == Cmd.CAM_TILT_YES:
        return False

    # Reducimos a solo 4 movimientos clave (0 -> Max -> Min -> 0)
    # Total tiempo: 0.15s (un 66% más rápido)
    secuencia = [TILT_MAX, TILT_MIN, 0]

    for angulo in secuencia:
        px.set_cam_tilt_angle(angulo)
        time.sleep(0.05) 

    px.last_cmd = Cmd.CAM_TILT_YES
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

def apply_safety(px, estado, st, det):
    px.last_state = estado

    # 1. PELIGRO EXTREMO (Mantiene el cambio de estado a SEARCH para huir)
    if px.distance_real < DANGER_DISTANCE and not st.is_escaping:
        log_event(px, px.last_state, f"🚨 Peligro extremo distancia={px.distance_real} → SCAPE")
        stop(px)
        backward(px)
        time.sleep(0.4)
        stop(px)
        st.is_escaping = True
        st.escape_end_time = time.time() + 1.0
        return Estado.SEARCH 

    # 2. ADVERTENCIA (Solo cambia la velocidad, NO el estado)
    if px.distance_real < WARNING_DISTANCE:
        # Solo logueamos si el cambio es nuevo para no inundar el log
        if not px.changed_speed_slow:
            log_event(px, px.last_state, f"⚠️ Velocidad lenta activa ({px.distance_real} cm)")
        px.changed_speed_slow = True
        # NOTA: No retornamos SEARCH aquí, dejamos que el flujo siga
    else:
        px.changed_speed_slow = False

    # 3. FUSIÓN PARA NEAR (Cambio de estado legítimo)
    if det.near_fused:
        log_event(px, px.last_state, "Baliza muy cerca según fusión → NEAR")
        stop(px)
        px.changed_speed_slow = False
        return Estado.NEAR
    
    # 4. FINALIZACIÓN DE ESCAPE
    if st.is_escaping and time.time() >= st.escape_end_time:
        log_event(px, px.last_state, "Maniobra de escape finalizada")
        st.is_escaping = False
        # Al salir de escape, solemos estar cerca, mantenemos precaución
        px.changed_speed_slow = True 
    
    return estado  # Retorna el estado original (TRACK, RECENTER, etc.)
    

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
    
    msg = (
        f"{prefix} f_lost={state.recenter_lost_frames} "
        f"search={det.valid_for_search} "
        f"near={det.valid_for_near} "
        f"centered={det.is_centered} "
        f"n={raw['n']} w={raw['w']} h={raw['h']} "
        f"area={det.area} x={raw['x']} y={raw['y']} "
        f"err_x={det.error_x} err_y={det.error_y}"
    )
    log_event(px, estado, msg)


def print_dashboard(px, estado, test_mode):
    os.system('clear')
    estado_name = estado.name if estado is not None else "NONE"
    print("="*45)
    print(f" 🐾 PICAR-X DASHBOARD | Estado actual: {estado_name} Estado anterior: {px.last_state.name}")
    print(f" 🐾 TEST MODE: {test_mode}")
    print("="*45)

    # Movimiento reportado por el último estado
    print(f" MOVIMIENTO: {px.last_cmd}")

    # Seguridad
    print(f" DISTANCIA:  {px.distance_real} cm " + ("⚠️ DANGER" if px.distance_real < DANGER_DISTANCE else "SAFE"))
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
    st.search_centered_frames = 0
    st.search_cam_dir = 1
    st.lost_in_space = False

    st.recenter_centered_frames = 0
    st.recenter_lost_frames = 0
    st.just_recentered = None

    st.track_lost_frames = 0
    st.track_centered_frames = 0

    st.near_hold_frames = 0
    st.near_backed = False
    st.near_cooldown = 0
    st.near_nodded = False

    st.is_escaping = False
    st.escape_end_time = 0
    st.last_sec_active = False

    px.last_state = Estado.RESET
    return Estado.SEARCH


def state_search(px, estado, st, distancia_real, test_mode):
    det, raw = get_detection(px)
    px.last_det = det

    # Entrada al estado (Idéntica a la tuya)
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, f"Entrando en SEARCH (test_mode={test_mode})")
        st.search_lost_frames = 0
        st.search_found_frames = 0
        st.search_centered_frames = 0
        st.search_cam_dir = 1
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0
        stop(px)
        time.sleep(0.1)
        px.last_state = Estado.SEARCH
        return Estado.SEARCH

    # Seguridad
    update_safety(px)
    estado = apply_safety(px, estado, st, det)
    if estado != Estado.SEARCH:
        px.last_state = Estado.SEARCH
        return estado

    # GESTIÓN DE DETECCIÓN
    # 1. Detección válida
    if det.valid_for_search or det.valid_for_near:
        st.search_lost_frames = 0

        # CENTRADO REAL
        if abs(det.error_x) <= 50:
            st.search_centered_frames += 1
        else:
            st.search_centered_frames = 0

        if st.search_centered_frames >= 2:
            log_event(px, Estado.SEARCH, "Centrado estable → RECENTER")
            px.last_state = Estado.SEARCH
            return Estado.RECENTER

        # PAN (sin huecos muertos)
        step = CAM_STEP
        if abs(det.error_x) > 120:
            step *= 3
        elif abs(det.error_x) > 80:
            step *= 2
        elif abs(det.error_x) > 10:
            step *= 1
        else:
            # error_x pequeño → no PAN
            px.last_state = Estado.SEARCH
            return Estado.SEARCH

        if det.error_x > 0:
            pan_right(px, step)
        else:
            pan_left(px, step)


    else:
        # SIN DETECCIÓN O RUIDO
        st.search_lost_frames += 1
        st.search_found_frames = 0
        
        # Plan B: Si llevamos mucho tiempo perdidos
        if st.search_lost_frames >= 30:
            log_event(px, Estado.SEARCH, "Búsqueda fallida → TRACK circular")
            st.lost_in_space = True
            px.last_state = Estado.SEARCH
            return Estado.TRACK

        # Barrido de cámara normal (Usando tus nuevas funciones con return 0/1)
        if st.search_cam_dir > 0:
            if pan_right(px) == 0:
                log_event(px, Estado.SEARCH, f"Valid_for_search={det.valid_for_search} Valid_for_near={det.valid_for_near} n={det.n} area={det.area} error_x={det.error_x} Tope DER -> Girando a IZQ")
                st.search_cam_dir = -1
        else:
            if pan_left(px) == 0:
                log_event(px, Estado.SEARCH, f"Valid_for_search={det.valid_for_search} Valid_for_near={det.valid_for_near} n={det.n} area={det.area} error_x={det.error_x} Tope IZQ -> Girando a DER")
                st.search_cam_dir = 1

    px.last_state = Estado.SEARCH
    return Estado.SEARCH


def state_recenter(px, estado, st, distancia_real, test_mode):

    # ============================================================
    # ENTRADA AL ESTADO RECENTER
    # ============================================================
    det, raw = get_detection(px)
    px.last_det = det

    if px.last_state != Estado.RECENTER:
        log_event(px, Estado.RECENTER, f"Entrando en RECENTER (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.RECENTER, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")
            return Estado.SEARCH

        st.recenter_centered_frames = 0
        st.recenter_lost_frames = 0

        px.changed_speed_slow = False  # RECENTER empieza en velocidad normal

        px.last_state = Estado.RECENTER
        return Estado.RECENTER


    # ============================================================
    # SEGURIDAD RECENTER
    # ============================================================
    update_safety(px)
    estado = apply_safety(px, estado, st, det)
    if estado != Estado.RECENTER:
        return estado

    # ============================================================
    # SIN DETECCIÓN → volver a SEARCH
    # ============================================================
    if not det.valid_for_search:
        st.recenter_lost_frames += 1
        if st.recenter_lost_frames >= 3:
            log_event(px, Estado.RECENTER, "Pérdida de detección → SEARCH")
            px.last_state = Estado.RECENTER
            return Estado.SEARCH
        px.last_state = Estado.RECENTER
        return Estado.RECENTER

    st.recenter_lost_frames = 0


    # ============================================================
    # ALINEACIÓN DEL CHASIS
    # ============================================================
    if abs(det.error_x) < 40:
        st.recenter_centered_frames += 1
        log_event(px, Estado.RECENTER, f"Frame estable ({st.recenter_centered_frames}/2)")
        if st.recenter_centered_frames >= 2:
            log_event(px, Estado.RECENTER, "Alineación suficiente → TRACK")
            px.last_state = Estado.RECENTER
            return Estado.TRACK
    else:
        # Solo reseteamos si nos salimos del margen de 40px
        st.recenter_centered_frames = 0

    # Comprobación de cercanía (ahora que sabemos que no hemos saltado a TRACK)
    if det.valid_for_near and px.last_state != Estado.NEAR:
        px.last_state = Estado.RECENTER
        return Estado.NEAR

    # Corrección suave del chasis
    log_event(px, Estado.RECENTER, f"Corrigiendo dirección, error_x={det.error_x}")

    if abs(det.error_x) >= 20:
        angulo = wheels_angle_error_x(px, det)
        log_event(px, Estado.RECENTER, f"Error significativo → wheels_angle_error_x (ángulo={angulo})")
        

    # ============================================================
    # MOVER CÁMARA PARA MANTENER LA BALIZA EN EL FRAME
    # ============================================================
    if det.error_x > 40:
        pan_right(px)
    elif det.error_x < -40:
        pan_left(px)


    if abs(det.error_x) > 80: # Si la baliza está ya en el tercio exterior del frame
        log_event(px, Estado.RECENTER, "Error demasiado grande para corregir avanzando → SEARCH")
        px.last_state = Estado.RECENTER
        return Estado.SEARCH

    # ============================================================
    # SALIDA RECENTER → TRACK
    # ============================================================
    # px.changed_speed_slow = True  # RECENTER se hace a velocidad lenta para mayor precisión
    forward(px)
    px.last_state = Estado.RECENTER
    return Estado.RECENTER


def state_track(px, estado, st, distancia_real, test_mode):

    # ============================================================
    # ENTRADA AL ESTADO TRACK
    # ============================================================
    det, raw = get_detection(px)
    px.last_det = det

    if px.last_state != Estado.TRACK:
        log_event(px, Estado.TRACK, f"Entrando en TRACK (test_mode={test_mode})")
        if test_mode:
            log_event(px, Estado.TRACK, "¡ATENCIÓN! MODO DE PRUEBAS ACTIVADO: el robot no se moverá.")
            return Estado.SEARCH

        st.track_centered_frames = 0
        st.track_lost_frames = 0
        px.changed_speed_slow = False

        px.last_state = Estado.TRACK
        return Estado.TRACK


    # ============================================================
    # SEGURIDAD TRACK
    # ============================================================
    update_safety(px)
    estado = apply_safety(px, estado, st, det)
    if estado != Estado.TRACK:
        return estado


    # ============================================================
    # PLAN B: si venimos de SEARCH con lost_in_space
    # ============================================================
    if st.lost_in_space:
        if det.valid_for_search:
            st.lost_in_space = False
            log_event(px, Estado.TRACK, "Referencia recuperada → TRACK")
            px.last_state = Estado.TRACK
            return Estado.TRACK
        else:
            log_event(px, Estado.TRACK, "Perdidos en el espacio → SEARCH")
            circle_robot(px, direction=1)

            px.last_state = Estado.TRACK
            return Estado.SEARCH


    # ============================================================
    # SIN DETECCIÓN → volver a SEARCH
    # ============================================================
    if not det.valid_for_search:
        st.track_lost_frames += 1
        if st.track_lost_frames >= 3:
            stop(px)
            log_event(px, Estado.TRACK, "Pérdida de detección → SEARCH")
            px.last_state = Estado.TRACK
            return Estado.SEARCH
        px.last_state = Estado.TRACK
        return Estado.TRACK

    st.track_lost_frames = 0


    # ============================================================
    # CORRECCIÓN DEL CHASIS (GIRO SUAVE LIMITADO)
    # ============================================================

    if abs(det.error_x) >= 20:
        raw_angle = det.error_x * 0.12
        servo_angle = round(clamp(raw_angle, -MAX_TRACK_ANGLE, MAX_TRACK_ANGLE), 1)

        px.set_dir_servo_angle(servo_angle)
        px.dir_current_angle = servo_angle

        log_event(px, Estado.TRACK, f"Corrigiendo dirección (ángulo={servo_angle})")

    else:
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0


    # ============================================================
    # MOVER CÁMARA PARA MANTENER LA BALIZA EN EL FRAME
    # ============================================================
    if abs(det.error_x) > 40:
        if det.error_x > 0:
            pan_right(px)
        else:
            pan_left(px)
    elif abs(det.error_x) < 15:
        # Opcional: Centrar suavemente la cámara si el robot ya está bien alineado
        # Esto prepara la cámara para la siguiente fase de RECENTER o NEAR
        pass

    # ============================================================
    # SI EL ERROR ES MUY GRANDE → NO AVANZAR
    # ============================================================
    if abs(det.error_x) > 80:
        stop(px)
        px.last_state = Estado.TRACK
        return Estado.TRACK


    # ============================================================
    # AVANCE CONTROLADO
    # ============================================================
    if abs(det.error_x) > 150:
        log_event(px, Estado.TRACK, "Error extremo → circle_robot + SEARCH")
        circle_robot(px, direction=1 if det.error_x > 0 else -1)
        px.last_state = Estado.TRACK
        return Estado.SEARCH

    forward(px)


    # ============================================================
    # SALIDA TRACK → NEAR
    # ============================================================
    if det.valid_for_near:
        log_event(px, Estado.TRACK, "Distancia crítica → NEAR")
        px.last_state = Estado.TRACK
        return Estado.NEAR

    px.last_state = Estado.TRACK
    return Estado.TRACK


def state_near(px, estado, st, distancia_real, test_mode):

    # ============================================================
    # ENTRADA AL ESTADO NEAR
    # ============================================================
    det, raw = get_detection(px)
    px.last_det = det

    if px.last_state != Estado.NEAR:
        log_event(px, Estado.NEAR, f"Entrando en NEAR (test_mode={test_mode})")

        st.near_hold_frames = 0
        st.near_backed = False
        st.near_nodded = False
        st.near_cooldown = 0

        stop(px)
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0
        time.sleep(0.1)
        px.changed_speed_slow = True

        px.last_state = Estado.NEAR
        return Estado.NEAR


    # ============================================================
    # SEGURIDAD NEAR
    # ============================================================
    update_safety(px)
    estado = apply_safety(px, estado, st, det)
    if estado != Estado.NEAR:
        px.last_state = Estado.NEAR
        return estado


    # ============================================================
    # RETROCESO (solo una vez)
    # ============================================================
    if px.distance_real < 15 and not st.near_backed:
        log_event(px, Estado.NEAR, "Retroceso preventivo")
        backward(px)
        time.sleep(0.1)
        stop(px)

        st.near_backed = True
        px.last_state = Estado.NEAR
        return Estado.NEAR


    # ============================================================
    # GESTO “SÍ” (solo una vez)
    # ============================================================
    if not st.near_nodded:
        log_event(px, Estado.NEAR, "Gesto de asentimiento")
        tilt_yes(px)
        
        st.near_nodded = True # Marcamos nodded como True para no repetir asentir
        st.near_backed = True # Marcamos backed como True para no retroceder después de asentir
        px.last_state = Estado.NEAR
        return Estado.NEAR

    # ============================================================
    # VERIFICACIÓN DE PRESENCIA (Si desaparece, volvemos a buscar)
    # ============================================================
    if not det.valid_for_search and not det.valid_for_near:
        # Si ya no la veo en absoluto, no tiene sentido esperar el cooldown
        log_event(px, Estado.NEAR, "Baliza desaparecida durante NEAR → SEARCH")
        px.last_state = Estado.NEAR
        return Estado.SEARCH


    # ============================================================
    # SALIDA DE NEAR (cooldown terminado)
    # ============================================================
    if st.near_nodded:
        st.near_cooldown += 1
        
        # Le damos un respiro. No evaluamos la salida hasta que pasen unos frames
        if st.near_cooldown > 20: 
            # CONDICIÓN DE ALEJAMIENTO REAL: 
            # Solo volvemos a RECENTER si el objeto está a más de 45cm (Margen de seguridad)
            # O si el error de centrado es EXTREMO (se ha ido del frame)
            if px.distance_real > 45 and abs(det.error_x) > 100:
                log_event(px, Estado.NEAR, f"Distancia real ({px.distance_real})cm Error_x ({det.error_x})-> RECENTER")
                st.near_cooldown = 0
                px.last_state = Estado.NEAR
                return Estado.RECENTER
            
            # Si llegamos aquí y el cooldown es muy alto (ej. 200), 
            # Paso a IDLE
            if st.near_cooldown > 200:
                 log_event(px, Estado.NEAR, "Misión cumplida: En espera estática")
                 stop(px) # Aseguramos frenado
                 px.last_state = Estado.NEAR
                 return Estado.IDLE

    px.last_state = Estado.NEAR
    return Estado.NEAR


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
    estado = init_internal_state(px)
    check_robot(px, log_event)
    state = RobotState()
    log_event(px, estado, "Inicio del sistema")
    ciclo_dashboard = 0

    try:
        while True:
            update_safety(px)

            if estado == Estado.IDLE:
                estado = state_idle(px, estado, state, px.distance_real, test_mode)

            elif estado == Estado.RESET:
                estado = state_reset(px, estado, state, px.distance_real, test_mode)

            elif estado == Estado.SEARCH:
                estado = state_search(px, estado, state, px.distance_real, test_mode)

            elif estado == Estado.RECENTER:
                estado = state_recenter(px, estado, state, px.distance_real, test_mode)

            elif estado == Estado.TRACK:
                estado = state_track(px, estado, state, px.distance_real, test_mode)

            elif estado == Estado.NEAR:
                estado = state_near(px, estado, state, px.distance_real, test_mode)

            ciclo_dashboard += 1
            if not test_mode and ciclo_dashboard % 5 == 0:
                print_dashboard(px, estado, test_mode)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n🛑 Interrupción detectada (Ctrl+C). Deteniendo robot...")
        
    finally:
        stop(px)
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
        print("✔ Robot detenido de forma segura.")


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
