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
        if not self.valid_for_search:
            return False
        return abs(self.error_x) <= 40

    @property
    def valid_for_search(self):
        if self.w == 0 or self.h == 0 or self.n == 0:
            return False

        return (
            self.n >= 1 and
            20 < self.w < 350 and     # coherente con Search
            20 < self.h < 480 and
            400 < self.area < 120000 and
            0 < self.x < 640 and
            0 < self.y < 480
        )
    
    @property
    def valid_for_near(self):
        if not self.valid_for_search:
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
            f"valid_for_near={self.valid_for_near}"
            ")"
        )

class RobotState:
    def __init__(self):

        # SEARCH
        self.search_no_det_frames = 0
        self.search_direction = 1 # 1 para derecha, -1 para izquierda

        # RECENTER
        self.recenter_centered_frames = 0
        self.recenter_lost_frames = 0

        # TRACK
        self.track_lost_frames = 0

        # Histeresis para NEAR 
        self.near_enter_frames = 0 
        self.near_exit_frames = 0
        self.near_done_backward = False
        self.near_lost_frames = 0

        # Cooldown tras backward en NEAR
        self.near_cooldown = None

        # Cooldown tras RECENTER
        self.just_recentered = None

        # Animación YES
        self.yes_step = 0          # En qué paso de la animación vamos
        self.yes_next_time = 0.0   # Cuándo toca ejecutar el siguiente paso
        self.near_did_yes = False  # Para que no se queje si lo llamas antes
        
        # SCAPE
        self.is_escaping = False
        self.escape_end_time = 0
        self.last_sec_active = False # Para detectar activación de seguridad en SEARCH

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
    # Logging
    px.last_log = None

    # Estado previo
    px.estado_actual = None
    px.last_state = None

    # Detección
    px.last_raw_n = 0

    # SEARCH
    px.search_cam_dir = 1      # Dirección del barrido de cámara en SEARCH  
    px.search_seen = 0

    # Cámara
    px.last_pan = 0
    px.last_tilt = 0

    # Direccion sevo angle
    px.dir_current_angle = 0

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
# ACCIONES BÁSICAS v2 — Movimiento fluido y continuo
# ============================================================

def stop(px):
    px.stop()
    # No tocamos el servo. La FSM decide cuándo centrarlo.

def forward(px, speed=FAST_SPEED):
    px.forward(speed)

def forward_slow(px, speed=SLOW_SPEED):
    px.forward(speed)

def backward(px, speed=SLOW_SPEED):
    px.backward(speed)
    # No bloqueamos la FSM. NEAR controla su propio backward.

# ------------------------------------------------------------
# GIRO CONTINUO REAL
# ------------------------------------------------------------

def turn_left(px, speed=TURN_SPEED):
    # Mantener el servo girado continuamente
    px.set_dir_servo_angle(SERVO_ANGLE_MIN)
    px.dir_current_angle = SERVO_ANGLE_MIN
    px.forward(speed)

def turn_right(px, speed=TURN_SPEED):
    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
    px.dir_current_angle = SERVO_ANGLE_MAX
    px.forward(speed)

# ------------------------------------------------------------
# SCAPE seguro (solo si se usa)
# ------------------------------------------------------------

def scape_danger(px, robot_state, speed=SLOW_SPEED):
    """
    Maniobra de evasión mejorada:
    - Retrocede
    - Gira según la cámara
    - Extiende escape si el obstáculo sigue cerca
    - Centra cámara y ruedas al terminar
    """

    # --- FASE 1: INICIO ---
    if not robot_state.is_escaping:
        log_event(px, "SEC", "¡ESCAPE ACTIVO! Retrocediendo...")

        # 1. Ángulo de escape más robusto
        if abs(px.last_pan) > 10:
            escape_angle = -px.last_pan
        else:
            escape_angle = SERVO_ANGLE_MIN  # giro fijo a la izquierda si la cámara estaba centrada

        target_angle = max(min(escape_angle, SERVO_ANGLE_MAX), SERVO_ANGLE_MIN)
        px.set_dir_servo_angle(target_angle)
        px.dir_current_angle = target_angle

        # 2. Retroceso inmediato
        px.backward(speed + 5)

        # 3. Señalización
        robot_state.is_escaping = True
        robot_state.last_sec_active = True
        robot_state.escape_end_time = time.time() + 1.2
        return False

    # --- FASE 2: MANTENER ESCAPE ---
    dist = update_safety(px)

    # Si el obstáculo sigue muy cerca, extendemos escape
    if 0 < dist < 15:
        robot_state.escape_end_time = time.time() + 0.3

    # --- FASE 3: FINALIZAR ---
    if time.time() >= robot_state.escape_end_time:
        px.stop()
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        # Cámara centrada para recuperación
        px.set_cam_pan_angle(0)
        px.last_pan = 0

        robot_state.is_escaping = False
        log_event(px, "SEC", "Maniobra terminada.")
        return True

    return False

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

def execute_motion(px, estado, cmd: Cmd, robot_state, test_mode=False):

    # ============================================================
    # MODO SIMULADO
    # ============================================================
    if test_mode:

        # --- Comandos permitidos en sim ---
        if cmd in (Cmd.CAM_PAN_LEFT, Cmd.CAM_PAN_RIGHT,
                   Cmd.CAM_TILT_TOP, Cmd.CAM_TILT_BOTTOM,
                   Cmd.STOP):

            # Ejecutar realmente (para ver movimiento en sim)
            try:
                if cmd == Cmd.STOP:
                    stop(px)
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
            scape_danger(px, robot_state, SLOW_SPEED)

        # --- Cámara ---
        elif cmd == Cmd.CAM_PAN_LEFT:
            pan_left(px)

        elif cmd == Cmd.CAM_PAN_RIGHT:
            pan_right(px)

        elif cmd == Cmd.CAM_TILT_TOP:
            tilt_top(px)

        elif cmd == Cmd.CAM_TILT_BOTTOM:
            tilt_bottom(px)
        elif cmd == Cmd.KEEP_ALIVE:
            # No hacemos nada, solo mantenemos la maniobra activa
            pass
        else:
            log_event(px, Estado.ERR, f"Comando desconocido: {cmd}")
            stop(px)
            return False

        log_event(px, estado, f"Ejecutado: {cmd.name}")
        return True

    except Exception as e:
        log_event(px, Estado.ERR, f"Error ejecutando {cmd}: {e}")
        stop(px)
        return False

# ============================================================
# SEGURIDAD
# ============================================================
def update_safety(px):
    distance = round(px.ultrasonic.read(), 2)

    # Filtro de valores basura
    if distance <= 0 or distance >= 400:   # ajusta 400 si tu sensor tiene otro rango
        d = 999
    else:
        d = distance
    return d


def apply_safety(px, estado, accion, state):

    d = update_safety(px)
    # --- NIVEL 2: FILTRO DE SEGURIDAD (SOBREESCRITURA) ---
    # Si el sensor detecta peligro o ya estamos en medio de un escape...
    if state.is_escaping or d < DANGER_DISTANCE:
        # Llamamos a escape. Si acaba de empezar, configurará motores a BACKWARD.
        terminado = scape_danger(px, state, SLOW_SPEED)
        
        # BLOQUEO CRÍTICO: Sobreescribimos la acción de la FSM
        # Usamos un comando neutro para que execute_motion no haga nada nuevo
        accion = Cmd.KEEP_ALIVE 
        
        if terminado:
            log_event(px, estado, f"[SEC] Zona segura ({d}cm). Reanudando...")
            estado = Estado.SEARCH # Tras el susto, buscamos de nuevo
    
    # Precaución: si no hay peligro inminente pero hay algo cerca, bajamos velocidad
    elif d < WARNING_DISTANCE and accion == Cmd.FORWARD:
        accion = Cmd.FORWARD_SLOW

    return estado, accion

# ============================================================
# FUNCIONES
# ============================================================

def get_detection(px):
    params = Vilib.detect_obj_parameter

    raw = {
        "color_x": params.get("color_x", -1),
        "color_y": params.get("color_y", -1),
        "color_w": params.get("color_w", 0),
        "color_h": params.get("color_h", 0),
        "color_n": params.get("color_n", 0),
    }

    det = Det(
        n = raw["color_n"],
        x = raw["color_x"],
        y = raw["color_y"],
        w = raw["color_w"],
        h = raw["color_h"]
    )

    # 🔥 Filtro anti-fantasma
    if det.n >= 1 and det.w == 0 and det.h == 0:
        det.n = 0

    # Log solo cuando aparece una detección nueva
    if raw["color_n"] > 0 and px.last_raw_n == 0:
        log_event(
            px, px.estado_actual,
            f"Det valid_for_search={det.valid_for_search} "
            f"is_centered={det.is_centered} "
            f"n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} "
            f"area={det.area} x={raw['color_x']} y={raw['color_y']} "
            f"error_x={det.error_x} error_y={det.error_y}"
        )

    px.last_raw_n = 1 if raw["color_n"] > 0 else 0

    return det, raw

def log_det(px, estado, det, raw, prefix=""):
    msg = (
        f"{prefix}"
        f"valid_for_search={det.valid_for_search} "
        f"valid_for_near={det.valid_for_near} "
        f"is_centered={det.is_centered} "
        f"n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} "
        f"area={det.area} x={raw['color_x']} y={raw['color_y']} "
        f"error_x={det.error_x} error_y={det.error_y}"
    )
    log_event(px, estado, msg)

def do_yes(px, robot_state):
    """
    Ejecuta el gesto de 'sí' de forma NO bloqueante.
    Devuelve True cuando la animación ha terminado por completo.
    """
    current_time = time.time()

    # Si aún no ha pasado el tiempo necesario para el siguiente movimiento, seguimos esperando
    if current_time < robot_state.yes_next_time:
        return False 

    # Definimos la secuencia de ángulos: (Arriba, Abajo, Centro) x 2
    secuencia = [
        TILT_MAX, TILT_MIN, 0,
        TILT_MAX, TILT_MIN, 0
    ]

    # Si todavía quedan pasos en la animación
    if robot_state.yes_step < len(secuencia):
        angulo = secuencia[robot_state.yes_step]
        px.set_cam_tilt_angle(angulo)
        
        # Preparamos el temporizador para el siguiente frame (ej. 0.15 seg de pausa)
        robot_state.yes_next_time = current_time + 0.15
        robot_state.yes_step += 1
        
        # Opcional: Log solo en el primer paso para no saturar
        if robot_state.yes_step == 1:
            log_event(px, px.estado_actual, "Iniciando gesto de 'SI' (no bloqueante)")
            
        return False # La animación sigue en curso

    else:
        # La animación ha terminado, reseteamos las variables por si queremos volver a usarla
        robot_state.yes_step = 0
        return True

def print_dashboard(px, estado, accion, dist, state):
    # Limpiar terminal (funciona en Linux/Raspberry Pi)
    os.system('clear')
    print("="*45)
    print(f" 🐾 PICAR-X DASHBOARD | Estado: {estado.name}")
    print("="*45)
    print(f" MOVIMIENTO: {accion.name}")
    print(f" DISTANCIA:  {dist} cm " + ("⚠️ DANGER" if dist < DANGER_DISTANCE else "✅ SAFE"))
    print("-"*45)
    print(f" SERVO DIR:  {px.dir_current_angle:>5.1f}°")
    print(f" CAM PAN:    {px.last_pan:>5.1f}°")
    print(f" ESCAPANDO:  {state.is_escaping}")
    print("="*45)
    print(" Presiona Ctrl+C para detener")

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
    px.dir_current_angle = 0
    px.last_pan = 0
    px.last_tilt = 0

    return Estado.SEARCH, Cmd.STOP

def state_search(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # ============================================================
    # 1. ENTRADA AL ESTADO
    # ============================================================
    if px.last_state != Estado.SEARCH:
        log_event(px, Estado.SEARCH, "Entrando en SEARCH")

        px.search_seen = 0
        robot_state.search_no_det_frames = 0
        robot_state.search_direction = 1  # giro inicial a la derecha

        # Cámara centrada al entrar
        px.set_cam_pan_angle(0)
        px.last_pan = 0

        px.last_state = Estado.SEARCH
        return Estado.SEARCH, Cmd.STOP


    # ============================================================
    # 2. RECUPERACIÓN TRAS SCAPE (MUY IMPORTANTE)
    # ============================================================
    if robot_state.last_sec_active:
        log_event(px, Estado.SEARCH, "[SEC] Recuperación tras evasión")

        # Reset de cámara
        px.set_cam_pan_angle(0)
        px.last_pan = 0

        # Reset de contadores
        robot_state.search_no_det_frames = 0
        px.search_seen = 0

        # Empezamos un barrido completo
        px.search_cam_dir = 1

        robot_state.last_sec_active = False
        return Estado.SEARCH, Cmd.CAM_PAN_RIGHT


    # ============================================================
    # 3. DETECCIÓN FIABLE
    # ============================================================
    is_reliable = det.valid_for_search and (60 < det.x < 580)
    is_in_edge = det.valid_for_search and not is_reliable

    if is_reliable:
        robot_state.search_no_det_frames = 0
        px.search_seen += 1

        # Si está centrada → RECENTER
        if abs(det.error_x) <= 40 and px.search_seen >= 3:
            return Estado.RECENTER, Cmd.STOP

        # Si no, corregimos con cámara
        return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT


    # ============================================================
    # 4. DETECCIÓN EN BORDE (NO DISPARAR PLAN B)
    # ============================================================
    if is_in_edge:
        # Solo corregimos con cámara, NO giramos el chasis todavía
        log_event(px, Estado.SEARCH, f"Baliza en borde ({det.x}) → corrigiendo cámara")
        robot_state.search_no_det_frames = max(robot_state.search_no_det_frames - 5, 0)
        return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT


    # ============================================================
    # 5. SIN DETECCIÓN → PLAN A / PLAN B
    # ============================================================
    px.search_seen = 0
    robot_state.search_no_det_frames += 1


    # ============================================================
    # 5A. PLAN B — GIRO DEL CHASIS
    # ============================================================
    if robot_state.search_no_det_frames > 80:

        # Ángulo de giro según la dirección guardada
        angulo_giro = SERVO_ANGLE_MAX * robot_state.search_direction
        px.set_dir_servo_angle(angulo_giro)
        px.dir_current_angle = angulo_giro

        # Barrido de cámara mientras gira (efecto radar)
        if px.last_pan >= PAN_MAX:
            px.search_cam_dir = -1
        elif px.last_pan <= PAN_MIN:
            px.search_cam_dir = 1

        return Estado.SEARCH, Cmd.FORWARD_SLOW


    # ============================================================
    # 5B. PLAN A — BARRIDO PAN SUAVE
    # ============================================================
    if px.last_pan >= PAN_MAX:
        px.search_cam_dir = -1
    elif px.last_pan <= PAN_MIN:
        px.search_cam_dir = 1

    return Estado.SEARCH, Cmd.CAM_PAN_RIGHT if px.search_cam_dir == 1 else Cmd.CAM_PAN_LEFT

def state_recenter(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # ------------------------------------------------------------
    # 1. Entrada al estado
    # ------------------------------------------------------------
    if px.last_state != estado:
        log_event(px, estado, "Entrando en RECENTER")
        robot_state.recenter_centered_frames = 0
        robot_state.recenter_lost_frames = 0

        # El chasis SIEMPRE debe empezar centrado
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0

        px.last_state = estado
        return Estado.RECENTER, Cmd.STOP


    # ------------------------------------------------------------
    # 2. Si NO hay detección válida → tolerar 5 frames
    # ------------------------------------------------------------
    if not det.valid_for_search:
        robot_state.recenter_lost_frames += 1
        if robot_state.recenter_lost_frames >= 5:
            log_det(px, estado, det, raw, prefix="RECENTER sin detección → SEARCH ")
            return Estado.SEARCH, Cmd.STOP
        return Estado.RECENTER, Cmd.STOP

    robot_state.recenter_lost_frames = 0


    # ------------------------------------------------------------
    # 3. Corrección horizontal (PAN)
    # ------------------------------------------------------------
    if abs(det.error_x) > 30:   # antes 40 → ahora más estricto
        robot_state.recenter_centered_frames = 0

        # Si la cámara está en límite, NO pasar a TRACK
        if px.last_pan == PAN_MAX and det.error_x > 0:
            return Estado.RECENTER, Cmd.STOP
        if px.last_pan == PAN_MIN and det.error_x < 0:
            return Estado.RECENTER, Cmd.STOP

        return Estado.RECENTER, Cmd.CAM_PAN_RIGHT if det.error_x > 0 else Cmd.CAM_PAN_LEFT


    # ------------------------------------------------------------
    # 4. PAN en límite (pero solo si realmente está centrado)
    # ------------------------------------------------------------
    if px.last_pan in (PAN_MAX, PAN_MIN):

        # Si error_x sigue siendo grande → NO pasar a TRACK
        if abs(det.error_x) > 60:
            log_event(px, estado, "PAN límite pero error_x grande → NO TRACK")
            robot_state.recenter_centered_frames = 0
            return Estado.RECENTER, Cmd.STOP

        # Si está realmente centrado → TRACK
        log_event(px, estado, "PAN límite + centrado → TRACK")
        px.set_cam_pan_angle(0)
        px.last_pan = 0
        robot_state.just_recentered = time.time()
        return Estado.TRACK, Cmd.FORWARD_SLOW


    # ------------------------------------------------------------
    # 5. Centrado normal
    # ------------------------------------------------------------
    robot_state.recenter_centered_frames += 1

    # Necesitamos más frames centrados para evitar falsos positivos
    if robot_state.recenter_centered_frames >= 4:
        log_event(px, estado, f"px.last_pan={px.last_pan} Alineado ✔ (cuerpo)")

        px.set_cam_pan_angle(0)
        px.last_pan = 0

        robot_state.just_recentered = time.time()
        return Estado.TRACK, Cmd.FORWARD_SLOW

    return Estado.RECENTER, Cmd.STOP


def state_track(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # Asegurar servo centrado al entrar en TRACK
    if px.last_state != Estado.TRACK:
        # No tocar la cámara al entrar en TRACK 
        # Solo centrar el servo de dirección
        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0
        log_event(px, estado, f"[ENTER] servo_angle={px.dir_current_angle} servo_pan={px.last_pan}")
        px.last_state = Estado.TRACK

    # Cooldown tras RECENTER
    if robot_state.just_recentered:
        if time.time() - robot_state.just_recentered < 0.3:
            return Estado.TRACK, Cmd.FORWARD_SLOW
        robot_state.just_recentered = None

    # 0. Si no hay detección → SEARCH
    if not det.valid_for_search:
        robot_state.track_lost_frames += 1
        if robot_state.track_lost_frames >= 3:
            log_det(px, Estado.TRACK, det, raw, prefix="Perdida baliza → SEARCH | ")
            return Estado.SEARCH, Cmd.STOP
        return Estado.TRACK, Cmd.FORWARD_SLOW
    robot_state.track_lost_frames = 0

    # 1. Si está cerca → NEAR
    if det.valid_for_near:
        robot_state.near_enter_frames += 1
        if robot_state.near_enter_frames >= 3:
            log_det(px, Estado.TRACK, det, raw, prefix="NEAR confirmado (3 frames) → NEAR | ")
            return Estado.NEAR, Cmd.STOP
        return Estado.TRACK, Cmd.FORWARD_SLOW
    robot_state.near_enter_frames = 0

    # 2. Corrección lateral proporcional (USA EL ERROR DE LA CÁMARA)
    if abs(det.error_x) > 40:
        # Mapeamos el error de la cámara (-320 a 320) al ángulo del servo (-30 a 30)
        # Un factor de 0.1 suele funcionar bien: 100px de error = 10 grados de giro
        KP = 0.15 
        target_angle = det.error_x * KP
        
        # Limitamos el ángulo para no forzar el servo
        target_angle = max(min(target_angle, SERVO_ANGLE_MAX), SERVO_ANGLE_MIN)
        
        px.set_dir_servo_angle(target_angle)
        px.dir_current_angle = target_angle
        log_event(px, estado, f"Corrección lateral: error_x={det.error_x} → servo_angle={px.dir_current_angle:.1f}")
        return Estado.TRACK, Cmd.FORWARD_SLOW

    # 3. Avance recto si está centrado
    px.set_dir_servo_angle(0)
    px.dir_current_angle = 0
    log_event(px, estado, f"[ENTER] servo_angle={px.dir_current_angle}")
    return Estado.TRACK, Cmd.FORWARD_SLOW

def state_near(px, estado, accion, robot_state):
    det, raw = get_detection(px)

    # --- 1. ENTRADA AL ESTADO (Frenado de Emergencia) ---
    if px.last_state != Estado.NEAR:
        log_event(px, Estado.NEAR, "Entrando en NEAR: ¡Frenando!")
        px.stop()  # Detenemos motores inmediatamente para mitigar inercia
        
        robot_state.near_done_backward = False
        robot_state.near_lost_frames = 0
        robot_state.near_exit_frames = 0
        robot_state.near_did_yes = False
        robot_state.near_cooldown = None

        px.set_dir_servo_angle(0)
        px.dir_current_angle = 0
        px.set_cam_tilt_angle(0)
        px.last_tilt = 0
        px.last_state = Estado.NEAR
        
        # Forzamos un ciclo de STOP para asegurar que los motores cortan
        return Estado.NEAR, Cmd.STOP

    # --- 2. VALIDACIÓN DE PRESENCIA ---
    if not det.valid_for_search:
        robot_state.near_lost_frames += 1
        if robot_state.near_lost_frames >= 5:
            log_event(px, Estado.NEAR, "Baliza perdida en NEAR → SEARCH")
            return Estado.SEARCH, Cmd.STOP
        return Estado.NEAR, Cmd.STOP
    robot_state.near_lost_frames = 0

    # --- 3. RETROCESO DE CORTESÍA (Espaciado) ---
    # Lo hacemos al principio para alejarnos del obstáculo/baliza antes del gesto
    if not robot_state.near_done_backward:
        log_event(px, Estado.NEAR, "Espaciado de seguridad (Backward)")
        robot_state.near_done_backward = True
        robot_state.near_cooldown = time.time() + 0.4 # Aumentamos un poco el tiempo
        return Estado.NEAR, Cmd.BACKWARD

    # Esperar a que el retroceso termine realmente
    if robot_state.near_cooldown:
        if time.time() < robot_state.near_cooldown:
            return Estado.NEAR, Cmd.KEEP_ALIVE # Mantenemos el BACKWARD activo
        else:
            px.stop() # Frenar tras el retroceso
            robot_state.near_cooldown = None
            return Estado.NEAR, Cmd.STOP

    # --- 4. GESTO DE RECONOCIMIENTO ("SÍ") ---
    if not robot_state.near_did_yes:
        terminado = do_yes(px, robot_state)
        if terminado:
            log_event(px, Estado.NEAR, "Gesto de 'SI' finalizado")
            robot_state.near_did_yes = True
        return Estado.NEAR, Cmd.STOP

    # --- 5. LÓGICA DE SALIDA O MANTENIMIENTO ---
    if not det.valid_for_near:
        robot_state.near_exit_frames += 1
        if robot_state.near_exit_frames >= 10: # Más margen para evitar tirones
            log_event(px, Estado.NEAR, "Baliza alejada → Volviendo a TRACK")
            return Estado.TRACK, Cmd.STOP
    else:
        robot_state.near_exit_frames = 0

    # Si la baliza se desplaza mucho lateralmente, corregimos solo con cámara
    if abs(det.error_x) > 100:
        return Estado.NEAR, Cmd.CAM_PAN_LEFT if det.error_x < 0 else Cmd.CAM_PAN_RIGHT

    return Estado.NEAR, Cmd.STOP

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
    estado, accion = init_internal_state(px)
    check_robot(px,log_event)
    state = RobotState()
    log_event(px, estado, "Inicio del sistema")

    while True:
        px.last_state = px.estado_actual
        px.estado_actual = estado
        distancia_real = update_safety(px) 

        # --- NIVEL 1: CÁLCULO DE INTENCIÓN (FSM) ---
        # Decidimos qué "querría" hacer el robot según la baliza
        if estado == Estado.IDLE: estado, accion = state_idle(px)
        elif estado == Estado.RESET: estado, accion = state_reset(px)
        elif estado == Estado.SEARCH: estado, accion = state_search(px, estado, accion, state)
        elif estado == Estado.RECENTER: estado, accion = state_recenter(px, estado, accion, state)
        elif estado == Estado.TRACK: estado, accion = state_track(px, estado, accion, state)
        elif estado == Estado.NEAR: estado, accion = state_near(px, estado, accion, state)

        # Actualiza la distancia y maneja la seguridad (puede modificar estado y accion)
        estado, accion = apply_safety(px, estado, accion, state) 

        # Solo llamamos a execute_motion UNA vez por ciclo
        execute_motion(px, estado, accion, state, test_mode)
        
        if not test_mode:
            print_dashboard(px, estado, accion, distancia_real, state)

        time.sleep(0.05) # Mayor frecuencia = respuesta más rápida

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
