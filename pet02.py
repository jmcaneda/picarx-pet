############################################################
# pet02.py
############################################################

import os
import time
from vilib import Vilib
from enum import Enum
from libs import hello_px, check_robot
from sound import sound_dog

# ============================================================
# CONSTANTES
# ============================================================

BALIZA_COLOR = "red"

PAN_MIN = -35
PAN_MAX = 35

TILT_MIN = -35
TILT_MAX = 35

SERVO_ANGLE_MIN = -30
SERVO_ANGLE_MAX = 30

CX = 320
CY = 240

FAST_SPEED = 20 
SLOW_SPEED = 5
TURN_SPEED = 1 

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
        return abs(self.error_x) <= 80

    @property
    def valid(self):
        return (
            self.n > 0 and
            200 < self.area < 20000
        )

    @property
    def valid_for_search(self):
        # Ignorar detecciones basura
        if self.w == 0 or self.h == 0:
            return False

        return (
            self.n >= 1 and
            20 < self.w < 300 and
            20 < self.h < 480 and
            800 < self.area < 80000 and
            0 < self.x < 640 and
            0 < self.y < 480
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

        # --- NUEVO: MEMORIA REAL ---
        self.search_valid_frames = 0
        self.recenter_centered_frames = 0
        self.track_lost_frames = 0
        
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

def init_internal_state(px):

    return Estado.IDLE, Cmd.STOP

def init_flags(px):
    px.last_log = None
    px.last_track_log = None
    px.estado_actual = None
    px.last_raw = {}
    px.last_state = None
    # px.last_cmd = None
    px.last_raw_n = 0
    px.last_sec = "safe"
    px.dist = 999
    px.search_dir = 1      # 1 = derecha, -1 = izquierda
    px.last_pan = 0
    px.last_paneo = None
    px.last_tilt = 0

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
    px.set_dir_servo_angle(SERVO_ANGLE_MIN)
    px.forward(speed)

def turn_right(px, speed=TURN_SPEED):
    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
    px.forward(speed)

def scape_danger(px, speed=SLOW_SPEED):
    px.set_dir_servo_angle(SERVO_ANGLE_MIN)
    px.backward(speed)
    time.sleep(0.5)
    px.set_dir_servo_angle(SERVO_ANGLE_MAX)
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

    # Crear objeto Det (solo una vez)
    det = Det(
        n = raw["color_n"],
        x = raw["color_x"],
        y = raw["color_y"],
        w = raw["color_w"],
        h = raw["color_h"]
    )

    # Log solo cuando aparece una detección nueva
    if raw["color_n"] > 0 and px.last_raw_n == 0:
        log_event(
            px, px.estado_actual,
            f"Det valid={det.valid} is_centered={det.is_centered} "
            f"n={raw['color_n']} w={raw['color_w']} h={raw['color_h']} "
            f"area={det.area} x={raw['color_x']} y={raw['color_y']}"
        )

    px.last_raw_n = 1 if raw["color_n"] > 0 else 0

    return det

def search_see(px, det):
    
    return Estado.TRACK, Cmd.STOP

def search_not_see(px):
    """
    SEARCH cuando NO hay detección válida:
    - Cámara centrada (mirada al frente)
    - Giro continuo del cuerpo
    - Ignorar detecciones basura
    - En cuanto aparezca una detección válida → RECENTER
    - Anti-bucle: invertir giro o resetear
    """

    # Parámetros
    CIRCLE_SWITCH_STEPS = 80      # invertir giro cada X pasos
    FULL_RESET_STEPS = 300        # hard reset si nada aparece

    # Inicialización defensiva
    if not hasattr(px, "search_steps"):
        px.search_steps = 0
    if not hasattr(px, "search_dir"):
        px.search_dir = 1

    px.search_steps += 1

    # --- 0. Hard reset anti-bucle ---
    if px.search_steps >= FULL_RESET_STEPS:
        log_event(px, Estado.SEARCH, "SEARCH hard reset → centrar cámara y reiniciar")
        px.search_steps = 0
        px.search_dir = 1
        px.last_pan = 0
        px.set_cam_pan_angle(0)
        return Estado.SEARCH, Cmd.WHEELS_TURN_RIGHT

    # --- 1. Cámara SIEMPRE centrada en modo búsqueda circular ---
    if px.last_pan != 0:
        px.last_pan = 0
        px.set_cam_pan_angle(0)

    # --- 2. Giro continuo del cuerpo ---
    # Invertir sentido cada cierto tiempo para evitar bucles
    if px.search_steps % CIRCLE_SWITCH_STEPS == 0:
        px.search_dir *= -1
        log_event(px, Estado.SEARCH, "SEARCH → invertir sentido de giro")

    if px.search_dir > 0:
        return Estado.SEARCH, Cmd.WHEELS_TURN_RIGHT
    else:
        return Estado.SEARCH, Cmd.WHEELS_TURN_LEFT

def do_yes(px):
    """
    Gesto doble de 'sí' acompañado de un sonido de perro.
    Se ejecuta una sola vez por evento.
    """
    try:
        # Sonido solo una vez
        sound_dog()

        for _ in range(2):
            # Gesto hacia arriba
            px.set_cam_tilt_angle(TILT_MAX)
            time.sleep(0.12)

            # Gesto hacia abajo
            px.set_cam_tilt_angle(TILT_MIN)
            time.sleep(0.12)

            # Volver al centro
            px.set_cam_tilt_angle(0)
            time.sleep(0.05)

    except Exception as e:
        print(f"[WARN] Error en gesto de 'sí': {e}")


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
    det = get_detection(px)

    # Entrada al estado
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

    # ------------------------------------------------------------
    # 1. Si hay detección válida → detener paneo y centrar
    # ------------------------------------------------------------
    if det.valid_for_search:

        # Acumular frames válidos
        px.search_seen += 1

        # Si está centrada → RECENTER
        if det.valid_for_search and det.is_centered and px.search_seen >= 2:
            log_event(px, Estado.SEARCH, "Baliza encontrada → RECENTER")
            return Estado.RECENTER, Cmd.STOP

        # Si no está centrada → corregir con cámara
        if det.error_x > 40:
            log_event(px, estado, "1 corregir con cámara → derecha")
            return Estado.SEARCH, Cmd.CAM_PAN_RIGHT

        if det.error_x < -40:
            log_event(px, estado, "2 corregir con cámara → izquierda")
            return Estado.SEARCH, Cmd.CAM_PAN_LEFT

        # Detección válida pero no centrada → no panees
        return Estado.SEARCH, Cmd.STOP

    # ------------------------------------------------------------
    # 2. Si NO hay detección válida → paneo
    # ------------------------------------------------------------
    else:
        px.search_seen = 0
        return search_not_see(px)

def state_recenter(px, dist, estado, accion, robot_state):
    det = get_detection(px)

    # Entrada al estado
    if px.last_state != estado:
        robot_state.recenter_centered_frames = 0
        robot_state.recenter_lost_frames = 0
        log_event(px, estado, "Entrando en RECENTER")
    px.last_state = estado

    # Seguridad
    estado, accion = apply_safety(px, dist, estado, accion)
    if estado != Estado.RECENTER:
        return estado, accion

    # ------------------------------------------------------------
    # 1. Si NO hay detección válida → tolerar 3 frames
    # ------------------------------------------------------------
    if not det.valid_for_search:
        robot_state.recenter_lost_frames += 1

        if robot_state.recenter_lost_frames >= 3:
            log_event(px, estado, "RECENTER sin detección → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        return Estado.RECENTER, Cmd.STOP

    # Si hay detección válida, resetear memoria
    robot_state.recenter_lost_frames = 0

    # ------------------------------------------------------------
    # 2. Corregir orientación del cuerpo
    # ------------------------------------------------------------
    if abs(det.error_x) > 40:
        robot_state.recenter_centered_frames = 0
        if det.error_x > 0:
            return Estado.RECENTER, Cmd.WHEELS_TURN_RIGHT
        else:
            return Estado.RECENTER, Cmd.WHEELS_TURN_LEFT

    # ------------------------------------------------------------
    # 3. Centrado → acumular frames
    # ------------------------------------------------------------
    robot_state.recenter_centered_frames += 1

    if robot_state.recenter_centered_frames >= 3:
        log_event(px, estado, "Alineado ✔ (cuerpo)")
        return Estado.TRACK, Cmd.FORWARD_SLOW

    return Estado.RECENTER, Cmd.STOP

def state_track(px, dist, estado, accion, robot_state):
    det = get_detection(px)

    # Entrada al estado
    if px.last_state != estado:
        log_event(px, estado, "Entrando en TRACK")
        robot_state.track_lost_frames = 0
        px.last_state = estado

    # --- Protección visual: si la baliza es demasiado grande, detener ---
    if det.valid_for_search and det.area > 35000:
        log_event(px, estado, f"Baliza muy cerca (área={det.area}) → BACKWARD")
        return Estado.TRACK, Cmd.BACKWARD

    if det.w > 200:
        log_event(px, estado, f"Baliza muy cerca (w={det.w}) → BACKWARD")
        return Estado.TRACK, Cmd.BACKWARD

    if det.h > 300:
        log_event(px, estado, f"Baliza muy cerca (h={det.h}) → BACKWARD")
        return Estado.TRACK, Cmd.BACKWARD

    # ------------------------------------------------------------
    # 1. Seguridad: si está demasiado cerca → STOP
    # ------------------------------------------------------------
    if dist <= DANGER_DISTANCE:
        log_event(px, estado, f"[SEC] CRITICAL: object < {dist} cm")
        return Estado.RESET, Cmd.SCAPE

    if dist <= SAFE_DISTANCE:
        log_event(px, estado, "Distancia segura alcanzada → STOP")
        do_yes(px)
        return Estado.IDLE, Cmd.STOP

    # ------------------------------------------------------------
    # 2. Si NO hay detección válida → tolerar 3 frames
    # ------------------------------------------------------------
    if not det.valid_for_search:
        robot_state.track_lost_frames += 1

        if robot_state.track_lost_frames >= 3:
            log_event(px, estado, "Perdida baliza → SEARCH")
            return Estado.SEARCH, Cmd.STOP

        # No perder TRACK inmediatamente
        return Estado.TRACK, Cmd.STOP

    # Si hay detección válida, resetear memoria
    robot_state.track_lost_frames = 0

    # ------------------------------------------------------------
    # 3. Corrección gruesa con ruedas (error grande)
    # ------------------------------------------------------------
    if abs(det.error_x) > 40:
        if det.error_x > 0:
            log_event(px, estado, f"Corrigiendo (ruedas) error_x={det.error_x}")
            # px.last_pan = 0
            # px.set_cam_pan_angle(0)
            return Estado.TRACK, Cmd.WHEELS_TURN_RIGHT
        else:
            log_event(px, estado, f"Corrigiendo (ruedas) error_x={det.error_x}")
            # px.last_pan = 0
            # px.set_cam_pan_angle(0)
            return Estado.TRACK, Cmd.WHEELS_TURN_LEFT

    # ------------------------------------------------------------
    # 4. Corrección fina con cámara (error pequeño)
    # ------------------------------------------------------------
    if abs(det.error_x) > 10:
        if det.error_x > 0:
            return Estado.TRACK, Cmd.CAM_PAN_RIGHT
        else:
            return Estado.TRACK, Cmd.CAM_PAN_LEFT

    # ------------------------------------------------------------
    # 5. Centrado → avanzar
    # ------------------------------------------------------------
    log_event(px, estado, "Avanzando hacia baliza")
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

        """
        if px.last_cmd != accion:
            log_event(px, estado, f"CMD {accion.name}")
        px.last_cmd = accion
        """

        estado, accion = apply_safety(px, px.dist, estado, accion)

        execute_motion(px, estado, accion, test_mode)

        time.sleep(0.1)

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
